/* Preserve explicit legacy workspace URLs while the plain root becomes the visitor page.
 * Always navigate to a fixed same-origin sibling; user input cannot set the destination. */
(() => {
  const current = new URL(window.location.href);
  if (!current.searchParams.has('ci') && current.searchParams.get('view') !== 'ide'
      && !current.searchParams.has('robot') && !current.searchParams.has('task') && current.hash !== '#ide') return;
  const destination = new URL('ide.html', current);
  destination.search = current.search;
  destination.hash = current.hash === '#ide' ? '' : current.hash;
  window.location.replace(destination.href);
})();
