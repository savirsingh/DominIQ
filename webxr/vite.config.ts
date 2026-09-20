import { iwsdkDev } from '@iwsdk/vite-plugin-dev';
import { defineConfig } from 'vite';

// Where the simulator runs: 'localhost' for the local docker stack, or 10.99.7.1 for the shared one
// over WireGuard (SIM_HOST=10.99.7.1 npm run dev).
const SIM_HOST = process.env.SIM_HOST ?? 'localhost';

// MJPEG camera ports, 8600 + 10 * slot (sim_config.ASSETS). The boat has no camera.
const CAMERAS: Record<string, number> = {
  quadcopter: 8600,
  'fixed-wing': 8610,
  'tower-1': 8630,
  'tower-2': 8640,
};

export default defineConfig({
  plugins: [iwsdkDev()],
  server: {
    // The headset loads this page over https, so plain-http streams and feeds on another host would
    // be blocked as mixed content. Proxying keeps every request same-origin.
    proxy: {
      // Position bridge (bridge/feed.py, or mission.py --webxr-feed), on this machine.
      '/feed': {
        target: process.env.FEED_TARGET ?? 'http://127.0.0.1:8781',
        rewrite: (path) => path.replace(/^\/feed/u, ''),
      },
      // Voice assistant sidecar (assistant/server.py). It spends API credits, so it only listens locally.
      '/ask': { target: process.env.ASSISTANT_TARGET ?? 'http://127.0.0.1:8782' },
      // /cam/<asset> -> http://<sim>:<port>/stream
      ...Object.fromEntries(
        Object.entries(CAMERAS).map(([name, port]) => [
          `/cam/${name}`,
          { target: `http://${SIM_HOST}:${port}`, rewrite: () => '/stream' },
        ]),
      ),
    },
  },
});
