var algorithms_landing_page =
[
    [ "Basics about Kalman Filtering and how it is used in INSTINCT", "KalmanFilterBasics.html", [
      [ "Basic relations", "KalmanFilterBasics.html#KalmanFilterBasics-Basic", [
        [ "Bayes' Theorem", "KalmanFilterBasics.html#KalmanFilterBasics-Bayes", null ],
        [ "Using Bayes' Theorem for Prediction", "KalmanFilterBasics.html#KalmanFilterBasics-Prediction", null ],
        [ "Using Bayes' Theorem for Update/Correction", "KalmanFilterBasics.html#KalmanFilterBasics-Update", null ],
        [ "Kalman filtering at a glance", "KalmanFilterBasics.html#KalmanFilterBasics-Summary", null ],
        [ "Square root implementation of the Kalman filter", "KalmanFilterBasics.html#KalmanFilterBasics-SQR", null ]
      ] ]
    ] ],
    [ "Single Point Positioning Basics", "SppBasics.html", [
      [ "Estimators", "SppBasics.html#SppBasics-estimators", null ],
      [ "Measurement model", "SppBasics.html#SppBasics-measurementModel", [
        [ "Measurement innovation", "SppBasics.html#SppBasics-measurementModel-innovation", [
          [ "Innovation vector", "SppBasics.html#SppBasics-measurementModel-innovation-vector", null ],
          [ "Measurement estimates", "SppBasics.html#SppBasics-measurementModel-innovation-estimates", null ]
        ] ],
        [ "Design matrix / Measurement sensitivity matrix", "SppBasics.html#SppBasics-measurementModel-sensitivityMatrix", null ],
        [ "Measurement error models", "SppBasics.html#SppBasics-measurementModel-measurementError", null ]
      ] ]
    ] ],
    [ "Single Point Positioning using (Weighed) Least Squares Estimation", "SppLSE.html", [
      [ "Algorithm - Linearized (Weighed) Least Squares Estimation", "SppLSE.html#SppLSE-LseAlgorithm", null ],
      [ "Unknowns", "SppLSE.html#SppLSE-unknowns", null ],
      [ "Measurement Model", "SppLSE.html#SppLSE-measurmentModel", [
        [ "Measurement residuals", "SppLSE.html#SppLSE-measurmentModel-residuals", null ],
        [ "Design matrix", "SppLSE.html#SppLSE-measurmentModel-designMatrix", null ],
        [ "Weight matrix", "SppLSE.html#SppLSE-measurmentModel-weightMatrix", null ]
      ] ]
    ] ],
    [ "Single Point Positioning using Kalman Filtering", "SppKF.html", [
      [ "Algorithm - Extended Kalman Filter", "SppKF.html#SppKF-KfAlgorithm", null ],
      [ "Unknowns", "SppKF.html#SppKF-unknowns", null ],
      [ "Process Model", "SppKF.html#SppKF-processModel", [
        [ "System Model", "SppKF.html#SppKF-processModel-systemModel", null ],
        [ "State transition matrix", "SppKF.html#SppKF-processModel-stateTransitionMatrix", null ],
        [ "Process noise covariance matrix", "SppKF.html#SppKF-processModel-processNoise", [
          [ "Van Loan method", "SppKF.html#SppKF-processModel-processNoise-vanLoan", null ],
          [ "Groves", "SppKF.html#SppKF-processModel-processNoise-groves", null ]
        ] ]
      ] ],
      [ "Measurement Model", "SppKF.html#SppKF-measurmentModel", [
        [ "Measurement innovation", "SppKF.html#SppKF-measurmentModel-innovation", null ],
        [ "Measurement sensitivity matrix", "SppKF.html#SppKF-measurmentModel-sensitivityMatrix", null ],
        [ "Measurement noise covariance matrix", "SppKF.html#SppKF-measurmentModel-measurementNoise", null ]
      ] ],
      [ "Initialization", "SppKF.html#SppKF-init", null ],
      [ "Inter system time difference reference system change", "SppKF.html#SppKF-InterChange", [
        [ "Case 1: New Sat System with higher priority found (GPS > GAL > GLO > ...)", "SppKF.html#SppKF-InterChange-Case1", null ],
        [ "Case 2: No observation for reference system within an epoch", "SppKF.html#SppKF-InterChange-Case2", null ],
        [ "Error propagation", "SppKF.html#SppKF-InterChange-ErrorProp", null ]
      ] ]
    ] ],
    [ "IMU Integrator (Earth-fixed frame)", "ImuIntegrator_e.html", [
      [ "Earth-fixed frame mechanization", "ImuIntegrator_e.html#ImuIntegrator-Mechanization-e", [
        [ "Attitude", "ImuIntegrator_e.html#ImuIntegrator-Mechanization-e-Attitude", [
          [ "Propagation of direction cosine matrix with time", "ImuIntegrator_e.html#ImuIntegrator-Mechanization-e-Attitude-DCM", null ],
          [ "Propagation of quaternion with time", "ImuIntegrator_e.html#ImuIntegrator-Mechanization-e-Attitude-Quaternion", null ]
        ] ],
        [ "Velocity", "ImuIntegrator_e.html#ImuIntegrator-Mechanization-e-Velocity", null ],
        [ "Position", "ImuIntegrator_e.html#ImuIntegrator-Mechanization-e-Position", null ]
      ] ],
      [ "Numerical Integration", "ImuIntegrator_e.html#ImuIntegrator-e-Numerical-Integration", [
        [ "Runge Kutta 4th order", "ImuIntegrator_e.html#ImuIntegrator-e-Numerical-Integration-RK4", null ]
      ] ]
    ] ],
    [ "IMU Integrator (local-navigation frame)", "ImuIntegrator_n.html", [
      [ "Local-navigation frame mechanization", "ImuIntegrator_n.html#ImuIntegrator-Mechanization-n", [
        [ "Attitude", "ImuIntegrator_n.html#ImuIntegrator-Mechanization-n-Attitude", [
          [ "Propagation of direction cosine matrix with time", "ImuIntegrator_n.html#ImuIntegrator-Mechanization-n-Attitude-DCM", null ],
          [ "Propagation of Euler angles with time", "ImuIntegrator_n.html#ImuIntegrator-Mechanization-n-Attitude-Euler", null ],
          [ "Propagation of quaternion with time", "ImuIntegrator_n.html#ImuIntegrator-Mechanization-n-Attitude-Quaternion", null ]
        ] ],
        [ "Velocity", "ImuIntegrator_n.html#ImuIntegrator-Mechanization-n-Velocity", null ],
        [ "Position", "ImuIntegrator_n.html#ImuIntegrator-Mechanization-n-Position", null ]
      ] ],
      [ "Appendix", "ImuIntegrator_n.html#ImuIntegrator-n-Appendix", [
        [ "Quaternion propagation comparison", "ImuIntegrator_n.html#ImuIntegrator-Mechanization-n-Attitude-Quaternion-Comparison", [
          [ "Titterton", "ImuIntegrator_n.html#ImuIntegrator-Mechanization-n-Attitude-Quaternion-Comparison-Titterton", null ],
          [ "Jekeli", "ImuIntegrator_n.html#ImuIntegrator-Mechanization-n-Attitude-Quaternion-Comparison-Jekeli", null ],
          [ "Groves", "ImuIntegrator_n.html#ImuIntegrator-Mechanization-n-Attitude-Quaternion-Comparison-Groves", null ]
        ] ]
      ] ]
    ] ],
    [ "INS/GNSS Loosely-coupled Kalman Filter (Earth-fixed frame)", "LooselyCoupledKF_e.html", [
      [ "Earth Frame Error State Equations", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-ErrorStateEquations", [
        [ "Short form", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-ErrorStateEquations-ShortForm", null ],
        [ "Detailed form", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-ErrorStateEquations-DetailedForm", null ]
      ] ],
      [ "Augmented State with Noise", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-Noise", [
        [ "Random Walk", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-Noise-RW", null ],
        [ "Gauss-Markov 1. Order", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-Noise-GM1", null ],
        [ "Noise scale matrix W", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-Noise-Scale-matrix", null ],
        [ "Groves' process noise definition", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-Groves-process-noise-definition", null ]
      ] ],
      [ "Kalman-Filter Matrices", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-KF-Matrices", [
        [ "State transition matrix 𝚽 & Process noise covariance matrix Q", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-KF-Phi-Q", [
          [ "State transition matrix 𝚽", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-KF-Phi", [
            [ "Exponential Matrix", "LooselyCoupledKF_e.html#autotoc_md10", null ],
            [ "Taylor series", "LooselyCoupledKF_e.html#autotoc_md11", null ]
          ] ],
          [ "Process noise covariance matrix Q", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-KF-Q", null ],
          [ "Van Loan method", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-KF-Phi-Q-Van-Loan", null ]
        ] ],
        [ "Error covariance matrix P", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-KF-P", null ],
        [ "Measurement innovation 𝛿z", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-deltaz", null ],
        [ "Measurement sensitivity Matrix H", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-KF-H", null ],
        [ "Measurement noise covariance matrix R", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-KF-R", null ]
      ] ],
      [ "Corrections", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-Corrections", [
        [ "Position, Velocity, Attitude", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-Corrections-State", null ],
        [ "Biases", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-Corrections-Bias", null ]
      ] ],
      [ "Appendix", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-Appendix", [
        [ "Derivation", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-ErrorStateEquations-Derivation", [
          [ "Position Equations", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-ErrorStateEquations-Derivation-Position", null ],
          [ "Attitude Equations", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-ErrorStateEquations-Derivation-Attitude", null ],
          [ "Velocity Equations", "LooselyCoupledKF_e.html#LooselyCoupledKF_e-ErrorStateEquations-Derivation-Velocity", null ]
        ] ]
      ] ]
    ] ],
    [ "INS/GNSS Loosely-coupled Kalman Filter (local-navigation frame)", "LooselyCoupledKF_n.html", [
      [ "Local-Level Frame Error State Equations", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-ErrorStateEquations-LocalLevelFrame", [
        [ "Short form", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-ErrorStateEquations-LocalLevelFrame-ShortForm", null ],
        [ "Detailed form", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-ErrorStateEquations-LocalLevelFrame-DetailedForm", null ]
      ] ],
      [ "Augmented State with Noise", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-Noise", [
        [ "Random Walk", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-Noise-RW", null ],
        [ "Gauss-Markov 1. Order", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-Noise-GM1", null ],
        [ "Noise scale matrix W", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-Noise-Scale-matrix", null ],
        [ "Groves' process noise definition", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-Groves-process-noise-definition", null ]
      ] ],
      [ "Kalman-Filter Matrices", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-KF-Matrices", [
        [ "State transition matrix 𝚽 & Process noise covariance matrix Q", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-KF-Phi-Q", [
          [ "State transition matrix 𝚽", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-KF-Phi", [
            [ "Exponential Matrix", "LooselyCoupledKF_n.html#autotoc_md12", null ],
            [ "Taylor series", "LooselyCoupledKF_n.html#autotoc_md13", null ]
          ] ],
          [ "Process noise covariance matrix Q", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-KF-Q", null ],
          [ "Van Loan method", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-KF-Phi-Q-Van-Loan", null ]
        ] ],
        [ "Error covariance matrix P", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-KF-P", null ],
        [ "Measurement innovation 𝛿z", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-deltaz", null ],
        [ "Measurement sensitivity Matrix H", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-KF-H", null ],
        [ "Measurement noise covariance matrix R", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-KF-R", null ]
      ] ],
      [ "Corrections", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-Corrections", [
        [ "Position, Velocity, Attitude", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-Corrections-State", null ],
        [ "Biases", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-Corrections-Bias", null ]
      ] ],
      [ "Unit discussion", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-Units", [
        [ "System matrix F", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-Units-F", null ],
        [ "Gauss-Markov constant 𝛽 & Noise scale matrix G", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-Units-beta-G", null ],
        [ "Process noise covariance matrix Q", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-Units-Q", null ],
        [ "Error covariance matrix P", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-Units-P", null ],
        [ "Measurement innovation 𝛿z", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-Units-deltaz", null ],
        [ "Measurement sensitivity Matrix H", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-Units-H", null ],
        [ "Measurement noise covariance matrix R", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-Units-R", null ]
      ] ],
      [ "Appendix", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-Appendix", [
        [ "Derivation", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-ErrorStateEquations-LocalLevelFrame-Derivation", [
          [ "Position Equations", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-ErrorStateEquations-LocalLevelFrame-Derivation-Position", null ],
          [ "Attitude Equations", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-ErrorStateEquations-LocalLevelFrame-Derivation-Attitude", null ],
          [ "Velocity Equations", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-ErrorStateEquations-LocalLevelFrame-Derivation-Velocity", null ]
        ] ],
        [ "Error Equations comparison", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-Appendix-ErrorStateEquations-LocalLevelFrame-Comparison", [
          [ "Titterton", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-Appendix-ErrorStateEquations-LocalLevelFrame-Comparison-Titterton", null ],
          [ "Gleason", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-Appendix-ErrorStateEquations-LocalLevelFrame-Comparison-Gleason", null ],
          [ "Noureldin", "LooselyCoupledKF_n.html#LooselyCoupledKF_n-Appendix-ErrorStateEquations-LocalLevelFrame-Comparison-Noureldin", null ]
        ] ]
      ] ]
    ] ],
    [ "IMU Simulator", "ImuSimulator.html", [
      [ "Acceleration Measurements", "ImuSimulator.html#ImuSimulator-Acceleration-Measurements", null ],
      [ "Angular Rate Measurements", "ImuSimulator.html#ImuSimulator-AngularRate-Measurements", null ],
      [ "Trajectory calculation", "ImuSimulator.html#ImuSimulator-Trajectory", [
        [ "Fixed", "ImuSimulator.html#ImuSimulator-Trajectory-Fixed", null ],
        [ "Linear", "ImuSimulator.html#ImuSimulator-Trajectory-Linear", null ],
        [ "Circular", "ImuSimulator.html#ImuSimulator-Trajectory-Circular", null ]
      ] ],
      [ "Flight angles", "ImuSimulator.html#ImuSimulator-Flight-Angles", [
        [ "Roll angle", "ImuSimulator.html#ImuSimulator-Flight-Angles-Roll", null ],
        [ "Pitch angle", "ImuSimulator.html#ImuSimulator-Flight-Angles-Pitch", null ],
        [ "Yaw angle", "ImuSimulator.html#ImuSimulator-Flight-Angles-Yaw", null ]
      ] ]
    ] ]
];