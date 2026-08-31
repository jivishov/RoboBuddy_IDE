import { EAGER_WALKING_POLICIES, LAZY_ROLLER_POLICIES, POLICY_FILES } from './contract.js';

// Resolve from this module rather than the site origin so GitHub Pages project
// deployments retain their repository path (for example, /RoboBuddy_IDE/).
const ASSET_ROOT = new URL('../../assets/microduck/', import.meta.url);
const ONNX_RUNTIME_ROOT = new URL('runtime/onnx/', ASSET_ROOT);
const ORT_MODULE_URL = new URL('ort.wasm.min.mjs', ONNX_RUNTIME_ROOT);

export class MicroDuckPolicyRuntime {
  constructor({ importOrt = () => import(ORT_MODULE_URL.href) } = {}) {
    this.importOrt = importOrt;
    this.sessions = new Map();
    this.loading = new Map();
    this.disposed = false;
  }

  async initialize() {
    this.ort = await this.importOrt();
    this.ort.env.wasm.wasmPaths = ONNX_RUNTIME_ROOT.href;
    this.ort.env.wasm.numThreads = 1;
    await Promise.all(EAGER_WALKING_POLICIES.map((name) => this.load(name)));
  }

  async ensureMode(mode) {
    if (mode === 'roller') await Promise.all(LAZY_ROLLER_POLICIES.map((name) => this.load(name)));
  }

  async load(name) {
    if (this.sessions.has(name)) return this.sessions.get(name);
    if (this.loading.has(name)) return this.loading.get(name);
    const promise = this.ort.InferenceSession.create(new URL(`policies/${POLICY_FILES[name]}`, ASSET_ROOT).href, { executionProviders: ['wasm'] }).then((session) => {
      this.loading.delete(name);
      if (this.disposed) { void session.release(); throw new Error('Policy runtime was disposed while loading.'); }
      this.sessions.set(name, session);
      return session;
    });
    this.loading.set(name, promise);
    return promise;
  }

  async infer(name, observation) {
    const session = await this.load(name);
    const inputName = session.inputNames[0];
    const outputs = await session.run({ [inputName]: new this.ort.Tensor('float32', observation, [1, 61]) });
    return Float32Array.from(outputs[session.outputNames[0]].data);
  }

  async dispose() {
    this.disposed = true;
    const sessions = [...this.sessions.values()];
    this.sessions.clear();
    await Promise.allSettled(sessions.map((session) => session.release()));
  }
}
