# MVP-3D TCP重测文档已更新

Stage MVP-3D-SINGLE-TCP-FIX 恢复单TCP客户端模型：

- 正常运行只允许ROS2桥接这一条持久TCP连接；
- 不再使用 `/mvp/stop`；
- 不再使用TCP probe参与正常验收；
- 不再使用多客户端线程。

请使用：

docs\MVP3D_SINGLE_TCP_RETEST.md
