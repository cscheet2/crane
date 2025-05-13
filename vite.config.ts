import checker from 'vite-plugin-checker';
import { defineConfig } from 'vite';
import path from 'path';

export default defineConfig({
  base: '/crane/',
  resolve: {
    alias: {
      '@': path.resolve(__dirname, 'src'),
    },
  },
  plugins: [
    checker({ typescript: true }),
  ],
});