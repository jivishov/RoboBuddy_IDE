// Small, local outline icons. No external font, script, or runtime dependency.
const paths = {
  link: '<path d="M10 13a5 5 0 0 0 7 .2l3-3a5 5 0 0 0-7-7l-1.7 1.7M14 11a5 5 0 0 0-7-.2l-3 3a5 5 0 0 0 7 7l1.7-1.7"/>',
  check: '<path d="m5 12 4 4L19 6"/>',
  refresh: '<path d="M20 7v5h-5M4 17v-5h5"/><path d="M6.1 6.1A8 8 0 0 1 20 12M4 12a8 8 0 0 0 13.9 5.9"/>',
  plug: '<path d="M8 3v5m8-5v5M6 8h12v3a6 6 0 0 1-12 0V8Zm6 9v4"/>',
  unplug: '<path d="m3 3 18 18M8 3v3m8-3v5M6 8h3m4 0h5v3a6 6 0 0 1-.7 2.8M6 11a6 6 0 0 0 9.1 5.1M12 17v4"/>',
  exit: '<path d="M9 4H5v16h4m7-13 5 5-5 5m-6-5h11"/>',
  power: '<path d="M12 3v9m-6.4-6a9 9 0 1 0 12.8 0"/>',
  powerOff: '<path d="m3 3 18 18M12 3v5m-6.4-2a9 9 0 0 0 12.7 12.7M18.4 6a9 9 0 0 1 2.5 8.1"/>',
  calibrate: '<circle cx="12" cy="12" r="7"/><path d="M12 2v5m0 10v5M2 12h5m10 0h5"/>',
  middle: '<path d="M4 4h5M4 4v5m16-5h-5m5 0v5M4 20h5m-5 0v-5m16 5h-5m5 0v-5"/><circle cx="12" cy="12" r="2"/>',
  save: '<path d="M5 3h12l4 4v14H3V3h2Zm2 0v6h10V3M7 21v-8h10v8"/>',
  cancel: '<path d="m6 6 12 12M6 18 18 6"/>',
  minus: '<path d="M5 12h14"/>',
  plus: '<path d="M5 12h14M12 5v14"/>',
  apply: '<path d="m9 6 6 6-6 6M4 12h11m5-7v14"/>',
  stop: '<path d="m7 3-4 4v10l4 4h10l4-4V7l-4-4H7Z"/><rect x="8" y="8" width="8" height="8" rx="1" fill="currentColor" stroke="none"/>',
};
export function icon(name) {
  if (!paths[name]) throw new Error('Unknown hardware icon: ' + name);
  return '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true" focusable="false">' + paths[name] + '</svg>';
}
export function iconButton(button, name, label, tooltip = label) {
  if (button.dataset.icon !== name) { button.innerHTML = icon(name); button.dataset.icon = name; }
  button.classList.add('icon-button');
  button.setAttribute('aria-label', label);
  button.title = tooltip;
  button.dataset.tooltip = tooltip;
}
