/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
module.exports = {
  tutorialSidebar: [
    {
      type: "doc",
      id: "index",                // docs/index.md
      label: "Welcome",
    },
    {
      type: "category",
      label: "Franka Arm",
      collapsible: true,
      collapsed: false,
      items: [
        "franka/overview",
        "franka/simulation_setup",
        "franka/pd_control",
        "franka/ros_integration",
      ],
    },
    {
      type: "link",
      label: "Asset API Reference",
      href: "/typedoc/",          // 예: typedoc 으로 API 문서 생성 시
    },
  ],
};
