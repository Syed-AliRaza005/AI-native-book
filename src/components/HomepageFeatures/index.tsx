import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  img: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Part 1: Foundations',
    img: require('@site/static/img/part1.png').default,
    description: (
      <>
        Learn the fundamental concepts of Physical AI, humanoid robotics, and the
        philosophy behind building intelligent machines that interact with the
        physical world.
      </>
    ),
  },
  {
    title: 'Part 2: Core Tooling',
    img: require('@site/static/img/part2.png').default,
    description: (
      <>
        Master the essential tools of the trade, including the Robot Operating
        System (ROS 2) and the Gazebo simulator for creating and testing your
        robotic systems.
      </>
    ),
  },
  {
    title: 'Part 3: Advanced AI',
    img: require('@site/static/img/part3.png').default,
    description: (
      <>
        Dive into advanced AI concepts with NVIDIA Isaac, exploring humanoid
        development, bipedal locomotion, and human-robot interaction.
      </>
    ),
  },
  {
    title: 'Part 4: Capstone Project',
    img: require('@site/static/img/part4.png').default,
    description: (
      <>
        Apply your knowledge to a capstone project: building a conversational
        robot that can understand and respond to natural language commands.
      </>
    ),
  },
];

function Feature({title, img, description}: FeatureItem) {
  return (
    <div className={clsx('col col--3')}>
      <div className="text--center">
        <img src={img} className={styles.featureImg} alt={title} />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
