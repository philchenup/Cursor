    SoSeparator* PclCloudToCoin(const ct::Cloud::Ptr& cloud, const float scale);

    /// 将点云 SoSeparator 显示到 viewer，不做碰撞检查。
    /// 若 sceneGroup 中已存在同名节点，先删除再更新显示。
    /// @param node 由 PclCloudToCoin() 生成的场景节点，传 nullptr 则只清除旧节点。
    /// @param name 场景图节点名，默认 "scenePcd"。
    void showViewerSeparator(SoSeparator* node, const char* name = "scenePcd");
