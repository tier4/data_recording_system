// docs/docusaurus.config.ts

import { themes as prismThemes } from "prism-react-renderer";
// Import the Docusaurus Config type definition
import type { Config } from "@docusaurus/types";

// Use the 'Config' type for autocompletion and type checking
const config: Config = {
  title: "Data Recording System",
  tagline: "Project Documentation",
  favicon: "images/favicon.ico",

  // Set the production URL of your site here
  url: "https://tier4.github.io",
  // Set the base path for your site
  // This is your repository name
  baseUrl: "/data_recording_system/",

  // GitHub Pages deployment config.
  organizationName: "tier4", // Your GitHub username
  projectName: "data_recording_system", // Your repository name

  onBrokenLinks: "throw",

  markdown: {
    hooks: {
      onBrokenMarkdownLinks: "warn",
    },
  },

  // Internationalization configuration
  i18n: {
    defaultLocale: "en",
    locales: ["ja", "en"],
  },

  presets: [
    [
      "classic",
      {
        docs: {
          // Point to the TypeScript sidebar file
          sidebarPath: "./sidebars.ts",
          // Optional: link to edit docs in your repo
          editUrl: "https://github.com/tier4/data_recording_system/tree/develop/r36.4.0/docs",
        },
        blog: false, // Optional: Disable the blog plugin
        theme: {
          customCss: "./src/css/custom.css",
        },
      } satisfies import("@docusaurus/preset-classic").Options,
    ],
  ],

  themeConfig: {
    // ... other theme config
    navbar: {
      title: "Data Recording System",
      logo: {
        alt: "My Site Logo",
        src: "images/logo.svg",
      },
      items: [
        {
          type: "docSidebar",
          sidebarId: "tutorialSidebar",
          position: "left",
          label: "Docs",
        },
        {
          href: "https://github.com/tier4/data_recording_system",
          label: "GitHub",
          position: "right",
        },
      ],
    },
    // ...
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies import("@docusaurus/preset-classic").ThemeConfig,
};

export default config;
