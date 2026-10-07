/* Progressive enhancement only. Guides and navigation remain readable without JavaScript. */
document.querySelectorAll('pre[data-copy]').forEach(pre => {
  const button = document.createElement('button');
  button.type = 'button'; button.className = 'copy'; button.textContent = 'Copy example';
  button.setAttribute('aria-label', `Copy ${pre.dataset.copy} example`);
  const status = document.createElement('span'); status.className = 'sr-only'; status.setAttribute('role', 'status');
  pre.before(button); button.after(status);
  button.addEventListener('click', async () => {
    try {
      if (!navigator.clipboard?.writeText) throw new Error('Clipboard unavailable');
      await navigator.clipboard.writeText(pre.textContent);
      button.textContent = 'Copied'; status.textContent = 'Example copied to clipboard.';
    } catch {
      const selection = window.getSelection(); const range = document.createRange(); range.selectNodeContents(pre);
      selection.removeAllRanges(); selection.addRange(range);
      status.textContent = 'Example selected. Use your browser’s Copy command.'; button.textContent = 'Selected — copy manually';
    }
    setTimeout(() => { button.textContent = 'Copy example'; }, 2200);
  });
});
document.querySelectorAll('[data-print]').forEach(button => button.addEventListener('click', () => window.print()));
document.querySelectorAll('.help-nav').forEach(menu => {
  menu.addEventListener('keydown', event => { if (event.key === 'Escape') { menu.open = false; menu.querySelector('summary').focus(); } });
  document.addEventListener('click', event => { if (!menu.contains(event.target)) menu.open = false; });
});
if ('IntersectionObserver' in window) {
  const observer = new IntersectionObserver(entries => {
    for (const entry of entries) if (entry.isIntersecting) {
      document.querySelectorAll('.toc a').forEach(link => {
        if (link.hash === '#' + entry.target.id) link.setAttribute('aria-current', 'location');
        else link.removeAttribute('aria-current');
      });
    }
  }, { rootMargin: '0px 0px -70% 0px' });
  document.querySelectorAll('.doc section[id]').forEach(section => observer.observe(section));
}
