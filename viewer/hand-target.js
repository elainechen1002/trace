const clamp = (value, minimum, maximum) => Math.min(maximum, Math.max(minimum, value));

export function mapHand(sample) {
  if (!sample || !Number.isFinite(sample.x_mm) || !Number.isFinite(sample.y_mm)) {
    return null;
  }

  return {
    right: clamp(sample.x_mm / 1000, -0.24, 0.24),
    up: clamp(sample.y_mm / 1000, 0.02, 0.42),
  };
}

export function watchHand(update) {
  watchFeed('/hand', (sample, enabled) => update(mapHand(sample), enabled));
}

export function watchIK(update) {
  watchFeed('/ik', update);
}

export function ikMessage(state) {
  if (!state?.tracking) return 'Show one hand to the Leap Motion';
  if (!state.active) return 'Pinch to guide the virtual gripper';
  if (state.error_mm > 4) return 'Target is outside the reachable pose';
  return 'Following your palm · Release to pause';
}

function watchFeed(path, update) {
  let enabled = false;

  async function poll() {
    try {
      const response = await fetch(path, {
        cache: 'no-store',
        signal: AbortSignal.timeout(600),
      });
      if (response.status === 404 && !enabled) return;
      if (!response.ok) throw new Error('Hand feed unavailable');
      enabled = true;
      update(await response.json(), enabled);
    } catch {
      update(null, enabled);
    }
    setTimeout(poll, 150);
  }

  poll();
}
