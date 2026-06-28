#ifndef ORB_SLAM3_DRIFT_STABILIZER_H
#define ORB_SLAM3_DRIFT_STABILIZER_H

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

namespace ORB_SLAM3 {

using StateVector = Eigen::VectorXf;

struct RuntimeFeatureSnapshot {
    double timestamp_s = 0.0;
    int tracked_map_points = 0;
    int inlier_matches = 0;
    float reprojection_error = 0.0f;
    float local_ba_residual = 0.0f;
    float velocity_magnitude = 0.0f;
    float imu_residual = 0.0f;
    bool relocalization_event = false;
    float tracking_confidence = 0.0f;
    float keyframe_rate_hz = 0.0f;
    float covisibility_degree = 0.0f;
    Eigen::Vector3f delta_translation = Eigen::Vector3f::Zero();
    Eigen::Vector3f delta_rotation = Eigen::Vector3f::Zero();
    Eigen::Vector3f accel_smoothness = Eigen::Vector3f::Zero();
    int tracking_state = 0;
};

struct Prediction {
    float predicted_drift_error = 0.0f;
    float risk_score = 0.0f;
    std::vector<float> latent_state;
    bool valid = false;
};

struct MPCParams {
    float q_pos = 1.0f;
    float q_rot = 1.0f;
    float r_input = 0.2f;
    float smoothness_gain = 0.2f;
    float max_translation_correction = 0.02f;
    float max_rotation_correction = 0.02f;
    float trajectory_emphasis = 0.6f;
};

struct CorrectionResult {
    Eigen::Vector3f translation = Eigen::Vector3f::Zero();
    Eigen::Vector3f rotation = Eigen::Vector3f::Zero();
    float confidence = 0.0f;
    bool valid = false;
};

struct CorrectionHistoryEntry {
    float predicted_error = 0.0f;
    float risk = 0.0f;
    float confidence = 0.0f;
    float applied_gain = 0.0f;
};

using CorrectionHistory = std::deque<CorrectionHistoryEntry>;

class TemporalSequenceBuffer {
public:
    explicit TemporalSequenceBuffer(size_t max_length);
    void Push(const StateVector& s);
    std::vector<StateVector> GetWindow() const;
    size_t Size() const;

private:
    const size_t max_length_;
    mutable std::mutex mutex_;
    std::deque<StateVector> window_;
};

class DriftPredictorGRU {
public:
    DriftPredictorGRU();
    bool Initialize(const std::string& model_path, bool use_cuda);
    Prediction Predict(const std::vector<StateVector>& seq);
    bool IsReady() const;

private:
    bool initialized_;
    std::string model_path_;
    bool use_cuda_;
};

class SACMetaController {
public:
    SACMetaController();
    MPCParams Adapt(const std::vector<float>& latent_state,
                    float predicted_error,
                    float risk_score,
                    const CorrectionHistory& history,
                    const MPCParams& base) const;
};

class RuntimeCsvLogger {
public:
    RuntimeCsvLogger();
    ~RuntimeCsvLogger();

    bool Open(const std::string& path);
    void Log(const RuntimeFeatureSnapshot& feat,
             const Prediction& pred,
             const CorrectionResult& corr,
             float applied_gain,
             bool correction_applied,
             bool loop_closing_active);
    void Close();

private:
    std::mutex mutex_;
    std::ofstream file_;
    bool header_written_;
};

struct DriftStabilizerOptions {
    bool enable = true;
    bool enable_csv_logging = true;
    bool enable_sac = false;
    bool use_cuda = false;
    bool deterministic_fallback = true;
    int sequence_length = 20;
    int min_sequence_length = 6;
    int sac_update_period_frames = 15;
    float risk_disable_threshold = 0.92f;
    float base_correction_gain = 0.15f;
    std::string model_path = "gru_mpc_model.onnx";
    std::string csv_path = "logs/drift_runtime_log.csv";
};

struct DriftStabilizerResult {
    bool correction_applied = false;
    float risk_score = 0.0f;
    float predicted_drift_error = 0.0f;
    float applied_gain = 0.0f;
    Sophus::SE3f correction = Sophus::SE3f();
};

class DriftStabilizerRuntime {
public:
    explicit DriftStabilizerRuntime(const DriftStabilizerOptions& options);

    DriftStabilizerResult Process(const RuntimeFeatureSnapshot& features,
                                  const Sophus::SE3f& current_pose,
                                  bool loop_closing_active);

private:
    StateVector BuildStateVector(const RuntimeFeatureSnapshot& f);
    CorrectionResult ComputeMpcCorrection(const Prediction& pred,
                                          const Sophus::SE3f& current_pose,
                                          const MPCParams& params) const;

private:
    DriftStabilizerOptions options_;
    TemporalSequenceBuffer buffer_;
    DriftPredictorGRU predictor_;
    SACMetaController sac_controller_;
    RuntimeCsvLogger logger_;
    MPCParams base_params_;
    MPCParams active_params_;
    CorrectionHistory history_;
    int frame_counter_;
    bool initialized_;
};

}  // namespace ORB_SLAM3

#endif
