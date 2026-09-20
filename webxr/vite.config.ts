import { iwsdkDev } from '@iwsdk/vite-plugin-dev';
import { defineConfig } from 'vite';

export default defineConfig({
  plugins: [iwsdkDev()],
  server: {
    // The position bridge (bridge/feed.py) runs on this machine. Proxying it keeps the page
    // same-origin, so the headset's https page never makes a mixed-content request.
    proxy: {
      '/feed': {
        target: process.env.FEED_TARGET ?? 'http://127.0.0.1:8781',
        rewrite: (path) => path.replace(/^\/feed/u, ''),
      },
    },
  },
});
