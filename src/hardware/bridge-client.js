export function helperAddress(value) {
  const url = new URL(value);
  if (!['http:', 'https:'].includes(url.protocol) || !['127.0.0.1', 'localhost'].includes(url.hostname) || url.username || url.password || url.search || url.hash || (url.pathname !== '/' && url.pathname !== '')) throw new Error('Use a loopback helper address such as http://127.0.0.1:8765.');
  return url.origin;
}

export class HardwareClient {
  constructor({onState = () => {}, onError = () => {}} = {}) {
    this.onState = onState; this.onError = onError;
    this.base = null; this.token = ''; this.session = null; this.sequence = 0;
    this.busy = false; this.inFlight = 0; this.generation = 0; this.polling = false; this.heartbeatPending = false;
  }
  async request(path, payload, {timeout = 6000, keepalive = false} = {}) {
    const response = await fetch(this.base + path, {
      method: payload === undefined ? 'GET' : 'POST', mode: 'cors', credentials: 'omit',
      headers: {Authorization: 'Bearer ' + this.token, ...(payload === undefined ? {} : {'Content-Type': 'application/json'})},
      ...(payload === undefined ? {} : {body: JSON.stringify(payload)}),
      signal: AbortSignal.timeout(timeout), keepalive
    });
    const data = await response.json();
    if (!response.ok) { const error = new Error(data.error?.message || 'Helper request failed.'); error.code = data.error?.code; throw error; }
    return data;
  }
  async pair(base, token) {
    this.close();
    const generation = this.generation;
    this.base = helperAddress(base);
    if (!token || token.length > 256) throw new Error('Enter the pairing code or open the local hardware panel.');
    this.token = token;
    const result = await this.request('/api/session', {});
    if (generation !== this.generation) throw new Error('A previous panel session response was ignored. Pair explicitly again.');
    if (result.state.apiVersion !== 'robobuddy.so101.hardware.v1') throw new Error('Hardware helper version mismatch.');
    this.session = result.session; this.sequence = 0;
    this.onState(result.state);
    this.heartbeatTimer = setInterval(() => this.heartbeat(), 800);
    this.pollTimer = setInterval(() => this.poll(), 250);
    return result.state;
  }
  async heartbeat() {
    if (!this.session || this.heartbeatPending) return;
    this.heartbeatPending = true;
    const generation = this.generation;
    try { const data = await this.request('/api/heartbeat', {session: this.session}); if (generation === this.generation) this.onState(data); }
    catch (error) { if (generation === this.generation) this.lost(error); }
    finally { if (generation === this.generation) this.heartbeatPending = false; }
  }
  async poll() {
    if (!this.session || this.polling) return;
    this.polling = true;
    const generation = this.generation;
    try { const data = await this.request('/api/state'); if (generation === this.generation) this.onState(data); }
    catch (error) { if (generation === this.generation) this.lost(error); }
    finally { if (generation === this.generation) this.polling = false; }
  }
  async ports() { return (await this.request('/api/ports')).ports; }
  async command(operation, args = {}, {priority = false, keepalive = false} = {}) {
    if (!this.session) throw new Error('Pair the local helper first.');
    if (this.busy && !priority) throw new Error('Wait for the current operation.');
    this.inFlight += 1; this.busy = true;
    const session = this.session;
    const generation = this.generation;
    try {
      const data = await this.request('/api/command', {session, sequence: ++this.sequence, issuedAtMs: Date.now(), operation, args}, {keepalive});
      if (generation !== this.generation) throw new Error('A previous panel session response was ignored. Check the current state.');
      this.onState(data);
      if (operation === 'disconnect') this.close();
      return data;
    } finally { if (generation === this.generation) {this.inFlight = Math.max(0, this.inFlight - 1); this.busy = this.inFlight > 0;} }
  }
  async stop() { return this.command('stop', {}, {priority: true}); }
  bestEffortStop() {
    if (this.session) void this.command('stop', {}, {priority: true, keepalive: true}).catch(() => {});
  }
  async quit() {
    if (!this.session) throw new Error('Pair the helper first.');
    await this.request('/api/shutdown', {session: this.session});
    this.close();
  }
  lost(error) {
    if (!this.session) return;
    this.bestEffortStop();
    this.close();
    this.onError(new Error('Helper communication lost. A physical stop is unconfirmed. Re-pair explicitly; check the arm locally. ' + error.message));
  }
  close() {
    clearInterval(this.heartbeatTimer); clearInterval(this.pollTimer);
    this.generation += 1;
    this.session = null; this.busy = false; this.inFlight = 0;
    this.polling = false; this.heartbeatPending = false;
  }
}

