import type { ReactNode } from 'react';
import Link from '@docusaurus/Link';

type FeatureItem = {
  title: string;
  img: string;
  description: ReactNode;
  link: string;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Module 0: Introduction to Physical AI & Robotics',
    img: require('@site/static/img/part1.png').default,
    description: (
      <>
        Learn the fundamental concepts of Physical AI, humanoid robotics, and the
        philosophy behind building intelligent machines that interact with the
        physical world.
      </>
    ),
    link: '/docs/introduction/vision-of-embodied-ai',
  },
  {
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    img: require('@site/static/img/part2.png').default,
    description: (
      <>
        Master the essential tools of the trade, including the Robot Operating
        System (ROS 2) and the Gazebo simulator for creating and testing your
        robotic systems.
      </>
    ),
    link: '/docs/module1-ros/architecture-communication',
  },
  {
    title: 'Module 2: The Digital Twin (Gazebo & Unity)',
    img: require('@site/static/img/part3.png').default,
    description: (
      <>
        Dive into advanced AI concepts with NVIDIA Isaac, exploring humanoid
        development, bipedal locomotion, and human-robot interaction.
      </>
    ),
    link: '/docs/module2-simulation/gazebo-setup-basics',
  },
  {
    title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
    img: require('@site/static/img/part4.png').default,
    description: (
      <>
        Apply your knowledge to a capstone project: building a conversational
        robot that can understand and respond to natural language commands.
      </>
    ),
    link: '/docs/module3-ai/omniverse-usd-isaac-sim',
  },
];

function Feature({ title, img, description, link }: FeatureItem) {
  return (
    <div className="w-full p-4 md:w-1/2 lg:w-1/4">
      <Link to={link} className="block h-full no-underline hover:no-underline group">
        <div className="h-full p-6 transition-all duration-300 border rounded-2xl bg-white/5 border-white/10 backdrop-blur-sm hover:transform hover:-translate-y-2 hover:bg-white/10 hover:shadow-xl hover:shadow-purple-500/20">
          <div className="relative mb-6 overflow-hidden rounded-xl aspect-video">
            <img
              src={img}
              alt={title}
              className="object-cover w-full h-full transition-transform duration-500 transform group-hover:scale-110"
            />
          </div>
          <h3 className="mb-3 text-xl font-bold text-white transition-colors group-hover:text-blue-400">
            {title}
          </h3>
          <p className="text-sm leading-relaxed text-slate-400 group-hover:text-slate-300">
            {description}
          </p>
        </div>
      </Link>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className="w-full">
      <div className="flex flex-wrap -m-4">
        {FeatureList.map((props, idx) => (
          <Feature key={idx} {...props} />
        ))}
      </div>
    </section>
  );
}
