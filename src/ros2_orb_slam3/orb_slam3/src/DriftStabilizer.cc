#include "DriftStabilizer.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <numeric>

namespace ORB_SLAM3 {

namespace {
inline float Clamp(float v, float lo, float hi) {
    return std::max(lo, std::min(v, hi));
}
}  // namespace

TemporalSequenceBuffer::TemporalSequenceBuffer(size_t max_length)
    : max_length_(max_length) {}

void TemporalSequenceBuffer::Push(const StateVector& s) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (window_.size() >= max_length_) {
        window_.pop_front();
    }
    window_.push_back(s);
}

std::vector<StateVector> TemporalSequenceBuffer::GetWindow() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::vector<StateVector>(window_.begin(), window_.end());
}

size_t TemporalSequenceBuffer::Size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return window_.size();
}

DriftPredictorGRU::DriftPredictorGRU()
    : initialized_(false), use_cuda_(false) {}

bool DriftPredictorGRU::Initialize(const std::string& model_path, bool use_cuda) {
    model_path_ = model_path;
    use_cuda_ = use_cuda;
    initialized_ = !model_path.empty();
    return initialized_;
}

Prediction DriftPredictorGRU::Predict(const std::vector<StateVector>& seq) {
    Prediction out;
    if (!initialized_ || seq.empty()) {
        return out;
    }

    const StateVector& s = seq.back();
    const int n = static_cast<int>(s.size());
    if (n < 6) {
        return out;
    }

    const float speed = std::abs(s[6]);
    const float inlier_ratio = Clamp(s[2], 0.0f, 1.0f);
    const float reproj = std::abs(s[3]);
    out.predicted_drift_error = 0.5f * reproj + 0.35f * speed + 0.15f * (1.0f - inlier_ratio);
    out.risk_score = Clamp(out.predicted_drift_error, 0.0f, 1.0f);
    out.latent_state = {speed, inlier_ratio, reproj};
    out.valid = true;
    return out;
}

bool DriftPredictorGRU::IsReady() const {
    return initialized_;
}

SACMetaController::SACMetaController() = default;

MPCParams SACMetaController::Adapt(const std::vector<float>& latent_state,
                                   float predicted_error,
                                   float risk_score,
                                   const CorrectionHistory& history,
                                   const MPCParams& base) const {
    MPCParams out = base;
    const float latent_speed = latent_state.empty() ? 0.0f : std::abs(latent_state.front());
    const float hist_conf = history.empty() ? 0.0f : history.back().confidence;
    const float stress = Clamp(0.5f * predicted_error + 0.4f * risk_score + 0.1f * latent_speed, 0.0f, 1.0f);

    out.q_pos = Clamp(base.q_pos * (1.0f + 0.4f * stress), 0.5f, 3.0f);
    out.q_rot = Clamp(base.q_rot * (1.0f + 0.5f * stress), 0.5f, 3.0f);
    out.r_input = Clamp(base.r_input * (1.0f + 0.7f * risk_score), 0.05f, 2.0f);
    out.smoothness_gain = Clamp(base.smoothness_gain * (1.0f + 0.6f * (1.0f - hist_conf)), 0.05f, 1.5f);
    out.max_translation_correction = Clamp(base.max_translation_correction * (1.0f - 0.3f * risk_score), 0.002f, 0.05f);
    out.max_rotation_correction = Clamp(base.max_rotation_correction * (1.0f - 0.2f * risk_score), 0.002f, 0.06f);
    out.trajectory_emphasis = Clamp(base.trajectory_emphasis + 0.2f * stress, 0.2f, 1.0f);
    return out;
}

RuntimeCsvLogger::RuntimeCsvLogger()
    : header_written_(false) {}

RuntimeCsvLogger::~RuntimeCsvLogger() {
    Close();
}

bool RuntimeCsvLogger::Open(const std::string& path) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (file_.is_open()) {
        return true;
    }
    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    file_.open(path, std::ios::out | std::ios::trunc);
    header_written_ = false;
    if (!file_.is_open()) {
        return false;
    }
    file_ << "timestamp,tracked_map_points,inlier_matches,inlier_ratio,reprojection_error,local_ba_residual,"
          << "delta_tx,delta_ty,delta_tz,delta_rx,delta_ry,delta_rz,velocity,imu_residual,relocalization_event,"
          << "tracking_confidence,keyframe_rate_hz,covisibility_degree,predicted_drift_error,risk_score,"
          << "corr_tx,corr_ty,corr_tz,corr_rx,corr_ry,corr_rz,correction_confidence,applied_gain,"
          << "correction_applied,loop_closing_active,tracking_state\n";
    header_written_ = true;
    return true;
}

void RuntimeCsvLogger::Log(const RuntimeFeatureSnapshot& feat,
                           const Prediction& pred,
                           const CorrectionResult& corr,
                           float applied_gain,
                           bool correction_applied,
                           bool loop_closing_active) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!file_.is_open() || !header_written_) {
        return;
    }
    const float inlier_ratio = (feat.tracked_map_points > 0)
                                   ? static_cast<float>(feat.inlier_matches) / static_cast<float>(feat.tracked_map_points)
                                   : 0.0f;
    file_ << std::fixed << std::setprecision(6)
          << feat.timestamp_s << ","
          << feat.tracked_map_points << ","
          << feat.inlier_matches << ","
          << inlier_ratio << ","
          << feat.reprojection_error << ","
          << feat.local_ba_residual << ","
          << feat.delta_translation.x() << "," << feat.delta_translation.y() << "," << feat.delta_translation.z() << ","
          << feat.delta_rotation.x() << "," << feat.delta_rotation.y() << "," << feat.delta_rotation.z() << ","
          << feat.velocity_magnitude << ","
          << feat.imu_residual << ","
          << (feat.relocalization_event ? 1 : 0) << ","
          << feat.tracking_confidence << ","
          << feat.keyframe_rate_hz << ","
          << feat.covisibility_degree << ","
          << pred.predicted_drift_error << ","
          << pred.risk_score << ","
          << corr.translation.x() << "," << corr.translation.y() << "," << corr.translation.z() << ","
          << corr.rotation.x() << "," << corr.rotation.y() << "," << corr.rotation.z() << ","
          << corr.confidence << ","
          << applied_gain << ","
          << (correction_applied ? 1 : 0) << ","
          << (loop_closing_active ? 1 : 0) << ","
          << feat.tracking_state << "\n";
}

void RuntimeCsvLogger::Close() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (file_.is_open()) {
        file_.close();
    }
}

DriftStabilizerRuntime::DriftStabilizerRuntime(const DriftStabilizerOptions& options)
    : options_(options),
      buffer_(static_cast<size_t>(std::max(2, options.sequence_length))),
      frame_counter_(0),
      initialized_(false) {
    initialized_ = predictor_.Initialize(options_.model_path, options_.use_cuda);
    if (options_.enable_csv_logging) {
        logger_.Open(options_.csv_path);
    }
    active_params_ = base_params_;
}

StateVector DriftStabilizerRuntime::BuildStateVector(const RuntimeFeatureSnapshot& f) {
    StateVector s(20);
    s.setZero();
    s(0) = static_cast<float>(f.tracked_map_points);
    s(1) = static_cast<float>(f.inlier_matches);
    s(2) = (f.tracked_map_points > 0)
               ? static_cast<float>(f.inlier_matches) / static_cast<float>(f.tracked_map_points)
               : 0.0f;
    s(3) = f.reprojection_error;
    s(4) = f.local_ba_residual;
    s(5) = f.delta_translation.norm();
    s(6) = f.velocity_magnitude;
    s(7) = f.imu_residual;
    s(8) = f.relocalization_event ? 1.0f : 0.0f;
    s(9) = f.tracking_confidence;
    s(10) = f.keyframe_rate_hz;
    s(11) = f.covisibility_degree;
    s(12) = f.delta_translation.x();
    s(13) = f.delta_translation.y();
    s(14) = f.delta_translation.z();
    s(15) = f.delta_rotation.x();
    s(16) = f.delta_rotation.y();
    s(17) = f.delta_rotation.z();
    s(18) = f.accel_smoothness.norm();
    s(19) = static_cast<float>(f.tracking_state);
    return s;
}

CorrectionResult DriftStabilizerRuntime::ComputeMpcCorrection(const Prediction& pred,
                                                              const Sophus::SE3f& current_pose,
                                                              const MPCParams& params) const {
    (void)current_pose;
    CorrectionResult out;
    if (!pred.valid) {
        return out;
    }

    const float damp = Clamp(1.0f - pred.risk_score, 0.0f, 1.0f);
    out.translation = Eigen::Vector3f(-params.q_pos * pred.predicted_drift_error * 0.01f, 0.0f, 0.0f);
    out.rotation = Eigen::Vector3f(0.0f, -params.q_rot * pred.predicted_drift_error * 0.005f, 0.0f);
    if (out.translation.norm() > params.max_translation_correction) {
        out.translation = out.translation.normalized() * params.max_translation_correction;
    }
    if (out.rotation.norm() > params.max_rotation_correction) {
        out.rotation = out.rotation.normalized() * params.max_rotation_correction;
    }
    out.confidence = Clamp(damp * params.trajectory_emphasis, 0.0f, 1.0f);
    out.valid = true;
    return out;
}

DriftStabilizerResult DriftStabilizerRuntime::Process(const RuntimeFeatureSnapshot& features,
                                                      const Sophus::SE3f& current_pose,
                                                      bool loop_closing_active) {
    DriftStabilizerResult out;
    if (!options_.enable) {
        return out;
    }

    buffer_.Push(BuildStateVector(features));
    const std::vector<StateVector> seq = buffer_.GetWindow();

    Prediction pred;
    if (initialized_ && static_cast<int>(seq.size()) >= options_.min_sequence_length) {
        pred = predictor_.Predict(seq);
    }

    if (options_.enable_sac && pred.valid && frame_counter_ % std::max(1, options_.sac_update_period_frames) == 0) {
        active_params_ = sac_controller_.Adapt(pred.latent_state, pred.predicted_drift_error, pred.risk_score, history_, base_params_);
    } else if (!options_.enable_sac) {
        active_params_ = base_params_;
    }

    const bool allow_correction = pred.valid && !loop_closing_active && pred.risk_score < options_.risk_disable_threshold;
    CorrectionResult corr;
    if (allow_correction) {
        corr = ComputeMpcCorrection(pred, current_pose, active_params_);
        out.correction_applied = corr.valid && corr.confidence > 0.05f;
        out.applied_gain = out.correction_applied ? options_.base_correction_gain * corr.confidence : 0.0f;
        if (out.correction_applied) {
            out.correction = Sophus::SE3f(Sophus::SO3f::exp(out.applied_gain * corr.rotation),
                                          out.applied_gain * corr.translation);
        }
    }

    out.risk_score = pred.risk_score;
    out.predicted_drift_error = pred.predicted_drift_error;

    history_.push_back({pred.predicted_drift_error, pred.risk_score, corr.confidence, out.applied_gain});
    while (history_.size() > 120) {
        history_.pop_front();
    }

    if (options_.enable_csv_logging) {
        logger_.Log(features, pred, corr, out.applied_gain, out.correction_applied, loop_closing_active);
    }

    ++frame_counter_;
    return out;
}

}  // namespace ORB_SLAM3
