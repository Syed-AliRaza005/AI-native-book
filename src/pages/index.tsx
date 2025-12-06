import type { ReactNode } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

function HeroSection() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className="relative overflow-hidden bg-slate-900 border-b border-white/10">
      {/* Background Gradient */}
      <div className="absolute inset-0 bg-gradient-to-br from-indigo-500/20 via-purple-500/10 to-pink-500/20 animate-pulse" />

      <div className="container relative z-10 px-4 py-24 mx-auto text-center lg:py-32">
        <h1 className="mb-6 text-5xl font-extrabold text-transparent bg-clip-text bg-gradient-to-r from-blue-400 to-pink-500 md:text-7xl animate-fade-in">
          {siteConfig.title}
        </h1>
        <p className="max-w-2xl mx-auto mb-10 text-xl text-slate-300 md:text-2xl animate-fade-in delay-100">
          Your comprehensive guide to building intelligent humanoid robots from the ground up.
        </p>

        <div className="flex flex-col items-center justify-center gap-4 sm:flex-row animate-fade-in delay-200">
          <Link
            className="px-8 py-3 text-lg font-semibold text-white transition-all duration-300 rounded-full bg-gradient-to-r from-blue-600 to-indigo-600 hover:from-blue-500 hover:to-indigo-500 hover:scale-105 shadow-lg shadow-blue-500/30"
            to="/docs/overview">
            Start Learning
          </Link>
          <div className="flex gap-4">
            <Link
              className="px-8 py-3 text-lg font-semibold text-white transition-all duration-300 border rounded-full border-white/20 bg-white/5 hover:bg-white/10 hover:border-white/40 backdrop-blur-sm"
              to="/docs/assessments/introduction">
              Assessments
            </Link>
            <Link
              className="px-8 py-3 text-lg font-semibold text-white transition-all duration-300 border rounded-full border-white/20 bg-white/5 hover:bg-white/10 hover:border-white/40 backdrop-blur-sm"
              to="https://github.com/Syed-AliRaza005/AI-native-book">
              GitHub
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={siteConfig.title}
      description="A Textbook by Panaversity for the Physical AI & Humanoid Robotics Course">
      <HeroSection />
      <main className="bg-slate-900 min-h-screen">
        <div className="container py-16">
          <HomepageFeatures />
        </div>
      </main>
    </Layout>
  );
}
