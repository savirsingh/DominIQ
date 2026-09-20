# Quest 3 WebXR map placement

This is a TypeScript + Meta IWSDK mixed-reality scene for Meta Quest 3. It uses the official IWSDK UIKitML/Horizon UI kit for its in-world settings panel and environment raycast system for surface placement.

When the user starts AR, the scene requires Quest's bounded-floor reference space (room-scale, not stationary mode) and uses passthrough with recognized room planes. Close the settings panel, aim either controller at a detected horizontal surface such as a table, and pull the index trigger to place the map (currently a green placeholder rectangle). **Confirm** locks it in and sets the app's `ready` state; **Undo** removes it so you can place it again. Once ready, three orbs wander across the map; get close to one or point a controller at it to see its beam and label. To access settings while in AR, squeeze either controller's grip button: an in-world Meta-style panel appears in front of you. Aim at **Re-scan room** and use the index trigger to open Quest's room-capture flow. Quest allows one room-capture request per AR session, so exit and restart AR before re-scanning again.

## Build system

The project uses [Vite](https://vite.dev/) with Meta's `@iwsdk/vite-plugin-dev` build integration. The runtime is `@iwsdk/core`, which is built on Three.js and provides UIKitML, XR input, locomotion, and environment raycasts. TypeScript's configuration is in `tsconfig.json`.

## Run locally

Install Node.js 20.19+ (or 22.12+) first, then from this directory:

```bash
npm install
npm run dev
```

Run the TypeScript check with:

```bash
npm run typecheck
```

Create the optimized production build with:

```bash
npm run build
```

Open the printed local URL in a desktop browser to preview. For Quest 3, open the HTTPS development URL in Meta Quest Browser, select **Start AR**, then close the settings panel, aim either controller at a detected surface and pull its trigger to place the map. The IWSDK environment-raycast system uses real-time WebXR hit testing and does not require a prior room scan.

The in-world **Room settings** panel is the official IWSDK UIKitML Horizon kit. **Re-scan room** calls Quest Browser's `XRSession.initiateRoomCapture()` while staying inside the active AR session; the panel itself is ray-interactable from controllers.

## Speech transcription

The in-world panel can show what you said. Set `VITE_ELEVENLABS_API_KEY` in `.env`, restart Vite, press **Speak** in the panel, talk, then press it again to stop. The recording is transcribed by ElevenLabs and the text appears in the panel. Speech is only displayed; it does not trigger any actions. The key is sent from the browser directly to ElevenLabs; use a restricted development key for this prototype.
