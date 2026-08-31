export class InferenceGate {
  constructor() { this.epoch = 0; this.sequence = 0; this.inFlight = false; }
  begin() {
    if (this.inFlight) return null;
    this.inFlight = true;
    return Object.freeze({ epoch: this.epoch, sequence: ++this.sequence });
  }
  accepts(tag) { return Boolean(tag) && this.inFlight && tag.epoch === this.epoch && tag.sequence === this.sequence; }
  finish(tag) { if (tag?.sequence === this.sequence) this.inFlight = false; }
  invalidate() { this.epoch += 1; this.sequence += 1; this.inFlight = false; }
}
