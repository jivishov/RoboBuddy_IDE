// Keep the existing storage IDs so saved light/dark preferences survive the visual refresh.
export const THEME_STORAGE_KEY = 'rbide.theme';
export const DEFAULT_THEME_ID = 'calm-light';

export const THEMES = Object.freeze({
  'calm-light': Object.freeze({
    id: 'calm-light',
    label: 'Paper',
    colorScheme: 'light',
    editorTheme: 'default',
    themeColor: '#f5f2eb',
  }),
  'midnight-teal': Object.freeze({
    id: 'midnight-teal',
    label: 'Graphite',
    colorScheme: 'dark',
    editorTheme: 'material-darker',
    themeColor: '#20221f',
  }),
});

export function resolveTheme(themeId) {
  return THEMES[themeId] || THEMES[DEFAULT_THEME_ID];
}

export function readStoredTheme() {
  try {
    return resolveTheme(localStorage.getItem(THEME_STORAGE_KEY)).id;
  } catch {
    return DEFAULT_THEME_ID;
  }
}

export function applyTheme(themeId, { persist = true } = {}) {
  const theme = resolveTheme(themeId);
  const root = document.documentElement;
  root.dataset.theme = theme.id;
  root.style.colorScheme = theme.colorScheme;
  document.querySelector('meta[name="theme-color"]')?.setAttribute('content', theme.themeColor);
  if (persist) {
    try { localStorage.setItem(THEME_STORAGE_KEY, theme.id); } catch { /* Storage is optional. */ }
  }
  return theme;
}
