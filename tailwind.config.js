/** @type {import('tailwindcss').Config} */
module.exports = {
    content: ["./src/**/*.{js,jsx,ts,tsx,md,mdx}"],
    theme: {
        extend: {
            colors: {
                primary: {
                    DEFAULT: '#2563eb', // Blue 600
                    dark: '#1d4ed8', // Blue 700
                    light: '#60a5fa', // Blue 400
                },
                secondary: {
                    DEFAULT: '#ec4899', // Pink 500
                    dark: '#db2777', // Pink 600
                },
                background: '#0f172a', // Slate 900
                surface: '#1e293b', // Slate 800
            },
            animation: {
                'fade-in': 'fadeIn 1s ease-out forwards',
                'float': 'float 6s ease-in-out infinite',
                'glow': 'glow 2s ease-in-out infinite alternate',
            },
            keyframes: {
                fadeIn: {
                    '0%': { opacity: '0', transform: 'translateY(10px)' },
                    '100%': { opacity: '1', transform: 'translateY(0)' },
                },
                float: {
                    '0%, 100%': { transform: 'translateY(0)' },
                    '50%': { transform: 'translateY(-20px)' },
                },
                glow: {
                    'from': { boxShadow: '0 0 10px rgba(37, 99, 235, 0.5)' },
                    'to': { boxShadow: '0 0 20px rgba(37, 99, 235, 0.8)' },
                },
            },
        },
    },
    plugins: [],
    darkMode: ['class', '[data-theme="dark"]'],
    corePlugins: {
        preflight: false, // Prepare for Docusaurus style conflict avoidance
    },
    blocklist: ['container'], // Avoid conflict with Docusaurus container class
};
