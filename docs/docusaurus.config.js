// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Robotics',
  tagline: 'Complete Professional Guide - From Beginner to Expert',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://book-hackathone.vercel.app',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/docs/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Minahil-Hamza', // Usually your GitHub org/user name.
  projectName: 'Speckit-plus-humanoid-robotics-', // Usually your repo name.

  onBrokenLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          routeBasePath: '/',
          sidebarPath: './sidebars.js',
          editUrl:
            'https://github.com/Minahil-Hamza/Speckit-plus-humanoid-robotics-/tree/main/docs/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/robotics-social-card.jpg',
      colorMode: {
        defaultMode: 'dark',
        respectPrefersColorScheme: false,
      },
      navbar: {
        title: 'Physical AI & Robotics',
        logo: {
          alt: 'Physical AI & Robotics Logo',
          src: 'img/robot-logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Learn',
          },
          {
            href: '/',
            label: 'Interactive Book',
            position: 'right',
          },
          {
            href: 'https://github.com/Minahil-Hamza/Speckit-plus-humanoid-robotics-',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Learning Path',
            items: [
              {
                label: 'Introduction to Robotics & AI',
                to: '/module-1',
              },
              {
                label: 'ROS2 Deep Dive',
                to: '/module-2',
              },
              {
                label: 'Computer Vision',
                to: '/module-3',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'Interactive Book',
                href: '/',
              },
              {
                label: 'GitHub Repository',
                href: 'https://github.com/Minahil-Hamza/Speckit-plus-humanoid-robotics-',
              },
            ],
          },
          {
            title: 'Author',
            items: [
              {
                label: 'Dr. Minahil Hamza',
                href: 'https://github.com/Minahil-Hamza',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Dr. Minahil Hamza. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: ['python', 'bash', 'cpp', 'cmake', 'yaml'],
      },
    }),
};

export default config;
