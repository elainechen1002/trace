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
  let enabled = false;

  async function poll() {
    try {
      const response = await fetch('/hand', {
        cache: 'no-store',
        signal: AbortSignal.timeout(600),
      });
      if (response.status === 404 && !enabled) return;
      if (!response.ok) throw new Error('Hand feed unavailable');
      enabled = true;
      update(mapHand(await response.json()), enabled);
    } catch {
      update(null, enabled);
    }
    setTimeout(poll, 150);
  }

  poll();
}
