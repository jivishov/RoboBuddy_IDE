export class IdeEditor {
  constructor(parent, { onChange, onSave, onRun, onCommandPalette, onCursor, theme = 'material-darker' }) {
    if (!window.CodeMirror) throw new Error('CodeMirror failed to load.');
    this.currentFile = 'main.py';
    this.executionHandle = null;
    this.suppress = false;
    this.cm = window.CodeMirror(parent, {
      value: '', mode: 'python', theme, lineNumbers: true,
      indentUnit: 4, tabSize: 4, indentWithTabs: false, lineWrapping: false,
      matchBrackets: true, autoCloseBrackets: true, foldGutter: true,
      gutters: ['CodeMirror-linenumbers', 'CodeMirror-foldgutter'],
      extraKeys: {
        'Ctrl-S': () => onSave?.(), 'Cmd-S': () => onSave?.(),
        'F5': () => onRun?.(),
        'Ctrl-Shift-P': () => onCommandPalette?.(), 'Cmd-Shift-P': () => onCommandPalette?.(),
        'Ctrl-/': 'toggleComment', 'Cmd-/': 'toggleComment',
      },
    });
    this.cm.on('change', () => { if (!this.suppress) onChange?.(this.currentFile, this.cm.getValue()); });
    this.cm.on('cursorActivity', () => onCursor?.(this.currentFile, this.getCursorLine(), this.cm.getCursor().ch + 1));
    this.cm.setSize('100%', '100%');
  }
  setFile(name, content) {
    this.currentFile = name;
    this.highlightLine(null);
    this.suppress = true;
    this.cm.setValue(String(content));
    this.cm.clearHistory();
    this.suppress = false;
    this.cm.focus();
  }
  getValue() { return this.cm.getValue(); }
  getCursorLine() { return this.cm.getCursor().line + 1; }
  getCursorColumn() { return this.cm.getCursor().ch + 1; }
  highlightLine(line) {
    if (this.executionHandle != null) this.cm.removeLineClass(this.executionHandle, 'background', 'cm-executing-line');
    this.executionHandle = null;
    if (!line) return;
    const index = Math.max(0, Math.min(this.cm.lineCount() - 1, Number(line) - 1));
    this.executionHandle = this.cm.addLineClass(index, 'background', 'cm-executing-line');
    this.cm.scrollIntoView({ line: index, ch: 0 }, 120);
  }
  focus() { this.cm.focus(); }
  refresh() { this.cm.refresh(); }
  setTheme(theme) { this.cm.setOption('theme', theme); this.cm.refresh(); }
  undo() { this.cm.undo(); }
  redo() { this.cm.redo(); }
  find() { this.cm.execCommand('find'); }
  replace() { this.cm.execCommand('replace'); }
  toggleComment() { this.cm.execCommand('toggleComment'); }
}
