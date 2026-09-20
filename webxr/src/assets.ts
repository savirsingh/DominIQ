import { AssetType, defineAssets } from '@iwsdk/core';

const publicAssetUrl = (path: string): string =>
  `${import.meta.env.BASE_URL}${path.replace(/^\/+/u, '')}`;

export default defineAssets({
  settings: {
    name: 'Room settings',
    type: AssetType.UIKitML,
    url: publicAssetUrl('ui/settings.uikitml'),
    priority: 'critical',
  },
  placement: {
    name: 'Map placement HUD',
    type: AssetType.UIKitML,
    url: publicAssetUrl('ui/placement.uikitml'),
    priority: 'critical',
  },
  orbLabel: {
    name: 'Orb name and description',
    type: AssetType.UIKitML,
    url: publicAssetUrl('ui/orb-label.uikitml'),
    priority: 'critical',
  },
});
