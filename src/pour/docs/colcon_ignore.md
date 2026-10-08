# colcon 构建配置

目录根部的 `COLCON_IGNORE` 用于让 colcon 跳过本目录及其子目录，避免纳入工作区构建。

需要恢复构建时，删除本目录的 `COLCON_IGNORE`，再从 `/ros2_ws` 执行 `colcon build`。
