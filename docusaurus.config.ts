import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Textbook by Syed Ali Raza',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://Syed-AliRaza005.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Syed-AliRaza005',
  projectName: 'AI-native-book', // Usually your repo name.
  trailingSlash: false,
  deploymentBranch: 'gh-pages',

  onBrokenLinks: 'throw',

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
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/Syed-AliRaza005/AI-native-book'
        },
        blog: false, // disabled blog
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Syed Ali Raza Logo',
        src: 'img/logo.svg',

      },
      items: [
        // {
        //   type: 'docSidebar',
        //   sidebarId: 'tutorialSidebar',
        //   position: 'left',
        //   label: 'Textbook',
        // },
        {
          to: '/docs/overview',
          label: 'Overview',
          position: 'left',
        },
        {
          href: 'https://github.com/Syed-AliRaza005/AI-native-book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Textbook',
          items: [
            {
              label: 'Overview',
              to: '/docs/overview',
            },
            {
              label: 'Learning Outcomes',
              to: '/docs/learning-outcomes',
            },
            {
              label: 'Hardware Requirements',
              to: '/docs/hardware-requirements',
            },
            {
              label: 'Assessments',
              to: '/docs/assessments/introduction',
            },

          ],
        },

        {
          title: 'Connect with Me ',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/Syed-AliRaza005/AI-native-book',
            },
            {
              label: 'Linkedin',
              href: 'https://www.linkedin.com/in/syed-ali-raza-hassan-6166a02ba/',
            },
            {
              label: 'Portfolio',
              href: 'https://portfolio-sarh.vercel.app/',
            },
            {
              label: 'Instagram',
              href: 'https://www.instagram.com/_itz_ali_005/',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Syed Ali Raza. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
