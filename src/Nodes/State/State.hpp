/*
    /// @brief Initialize the State with Imu data
    /// @param[in] state Partial state data
    /// @param[in] linkId Id of the link over which the data is received
    void initAttitude(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Initialize the State with Gnss data
    /// @param[in] state Partial state data
    /// @param[in] linkId Id of the link over which the data is received
    void initPositionVelocity(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Finalize the initialization
    /// @param[in] currentTime The latest time
    void finalizeInit(const InsTime& currentTime);

    /// @brief Update the current State
    /// @param[in] state The new state
    /// @param[in] linkId Id of the link over which the data is received
    void updateState(const std::shared_ptr<NodeData>& state, ax::NodeEditor::LinkId linkId);

    /// The initial vehicle state
    StateData initialState;

    /// The current vehicle state
    StateData currentState;

    bool dynamicStateInit = true;
    std::array<float, 3> initLatLonAlt{ 0, 0, 0 };
    std::array<float, 3> initRollPitchYaw{ 0, 0, 0 };
    std::array<float, 3> initVelocityNED{ 0, 0, 0 };

    double initDuration = 5;
    InsTime averageStartTime;

    double countAveragedAttitude = 0;
    double countAveragedPosition = 0;
    double countAveragedVelocity = 0;
    */