import { MICRODUCK_COMMANDS, commandError, validateCommand } from './command-catalog.js';
import { deepFreeze } from './contract.js';

export class MicroDuckCommandBus {
  constructor({ now = () => performance.now(), onApply = () => {}, onPreempt = () => {}, onExpire = () => {} } = {}) {
    this.now = now;
    this.onApply = onApply;
    this.onPreempt = onPreempt;
    this.onExpire = onExpire;
    this.lease = null;
    this.owned = new Set();
    this.values = Object.fromEntries(Object.entries(MICRODUCK_COMMANDS).filter(([, item]) => item.classification === 'continuous').map(([name, item]) => [name, structuredClone(item.neutral)]));
    this.values.sound = { tag: null, hold: false };
  }

  execute(name, input = {}, { source = 'human', controllerId = source, durationMs } = {}, mode = 'walking') {
    const definition = MICRODUCK_COMMANDS[name];
    const sourceRule = definition?.authority?.[source];
    if (!sourceRule) throw commandError('INVALID_ARGUMENT', `Unknown controller source: ${source}`);
    const validated = validateCommand(name, input, mode);
    this.enforceAuthority(name, definition, { source, controllerId });
    const inactiveAudio = (name === 'theremin' || name === 'chorale') && input.active === false;
    const heldWheee = name === 'sound' && input.tag === 'wheee' && input.hold === true;
    if ((definition.classification === 'continuous' && !inactiveAudio) || heldWheee) {
      this.acquire(name, definition, { source, controllerId, durationMs });
      this.values[name] = structuredClone(validated.applied);
      this.owned.add(name);
    } else if (inactiveAudio) {
      this.values[name] = structuredClone(definition.neutral);
      this.owned.delete(name);
    } else if (name === 'look') {
      this.prepareIntentWrite('head', definition, { source, controllerId });
      this.values.head = structuredClone(validated.applied.solvedHead);
    } else if (name === 'sound') {
      this.values.sound = { tag: validated.applied.tag, hold: false };
    } else if (name === 'stop') {
      this.values.move = structuredClone(MICRODUCK_COMMANDS.move.neutral);
      this.owned.delete('move');
    }
    this.onApply(name, validated.applied, { source, controllerId });
    return deepFreeze({ ok: true, command: name, ...validated, owner: this.lease ? { source: this.lease.source, controllerId: this.lease.controllerId } : null });
  }

  enforceAuthority(name, definition, { source, controllerId }) {
    this.expire();
    if (name === 'get_state' || name === 'get_mode' || !this.lease) return;
    const current = this.lease;
    const same = current.source === source && current.controllerId === controllerId;
    if (same) return;
    if (definition.authority[source].priority <= definition.authority[current.source].priority) throw commandError('COMMAND_CONFLICT', `${current.source} controller already owns the MicroDuck command lease.`);
    this.onPreempt({ ...current, owned: [...this.owned] }, { source, controllerId, command: name });
    this.neutralizeOwned();
    this.lease = null;
  }

  acquire(name, definition, { source, controllerId, durationMs }) {
    this.expire();
    const current = this.lease;
    const same = current && current.source === source && current.controllerId === controllerId;
    const sourceRule = definition.authority[source];
    if (current && !same && sourceRule.priority <= definition.authority[current.source].priority) throw commandError('COMMAND_CONFLICT', `${current.source} controller already owns the MicroDuck command lease.`);
    if (current && !same) {
      this.onPreempt({ ...current, owned: [...this.owned] }, { source, controllerId, command: name });
      this.neutralizeOwned();
    }
    let requested = durationMs;
    if (sourceRule.durationRequired && (!Number.isFinite(requested) || requested < sourceRule.minimumDurationMs || requested > sourceRule.maximumDurationMs)) throw commandError('INVALID_ARGUMENT', `${source} ${name} requires durationMs from ${sourceRule.minimumDurationMs} through ${sourceRule.maximumDurationMs}.`);
    if (!Number.isFinite(requested)) requested = sourceRule.defaultDurationMs;
    const bounded = Math.max(sourceRule.minimumDurationMs, Math.min(sourceRule.maximumDurationMs, requested));
    this.lease = { source, controllerId, deadline: this.now() + bounded };
  }

  prepareIntentWrite(name, definition, { source, controllerId }) {
    this.expire();
    if (!this.lease) return;
    const same = this.lease.source === source && this.lease.controllerId === controllerId;
    if (same) { this.owned.delete(name); return; }
    if (definition.authority[source].priority <= definition.authority[this.lease.source].priority) throw commandError('COMMAND_CONFLICT', `${this.lease.source} controller already owns the MicroDuck command lease.`);
    this.onPreempt({ ...this.lease, owned: [...this.owned] }, { source, controllerId, command: name });
    this.neutralizeOwned();
    this.lease = null;
  }

  refresh({ source, controllerId, durationMs = 5000 } = {}) {
    this.expire();
    if (!this.lease || this.lease.source !== source || this.lease.controllerId !== controllerId) return false;
    const rule = MICRODUCK_COMMANDS.move.authority[source];
    const bounded = Math.max(rule.minimumDurationMs, Math.min(rule.maximumDurationMs, Number(durationMs) || rule.defaultDurationMs));
    this.lease.deadline = this.now() + bounded;
    return true;
  }

  connect({ source, controllerId, durationMs } = {}) {
    this.acquire('connect', MICRODUCK_COMMANDS.move, { source, controllerId, durationMs });
    return this.snapshot().lease;
  }

  expire(at = this.now()) {
    if (this.lease && at >= this.lease.deadline) {
      const expired = { ...this.lease, owned: [...this.owned] };
      this.neutralizeOwned();
      this.lease = null;
      this.onExpire(expired);
      return true;
    }
    return false;
  }

  cancel({ source, controllerId } = {}) {
    if (!this.lease) return false;
    if (source && (this.lease.source !== source || (controllerId && this.lease.controllerId !== controllerId))) return false;
    this.neutralizeOwned();
    this.lease = null;
    return true;
  }

  ownedCommandsFor(source, controllerId) {
    this.expire();
    if (!this.lease || this.lease.source !== source || (controllerId && this.lease.controllerId !== controllerId)) return [];
    return [...this.owned];
  }

  neutralizeOwned() {
    for (const name of this.owned) {
      const definition = MICRODUCK_COMMANDS[name];
      this.values[name] = structuredClone(definition.neutral ?? definition.heldNeutral);
    }
    this.owned.clear();
  }

  snapshot() {
    this.expire();
    return deepFreeze({ lease: this.lease ? { ...this.lease } : null, values: structuredClone(this.values) });
  }

  isOwnedBy(source, controllerId) {
    this.expire();
    return Boolean(this.lease && this.lease.source === source && this.lease.controllerId === controllerId);
  }
}
