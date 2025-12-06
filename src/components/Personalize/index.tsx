import type { ReactNode } from 'react';

// This component was causing build errors due to missing UserStateProvider.
// It appears to be unused. Replaced with null/pass-through for now.
export default function Personalize({ children }: { children?: ReactNode }): ReactNode {
  return <>{children}</>;
}
