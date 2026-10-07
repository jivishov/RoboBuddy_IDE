import { expect, test } from '@playwright/test';

test('vendored ORT matches walking and roller CPU fixtures', async ({ page }) => {
  await page.goto('/');
  const result = await page.evaluate(async () => {
    const fixture = await (await fetch('./assets/microduck/fixtures/inference-parity.json')).json();
    const ort = await import('./assets/microduck/runtime/onnx/ort.wasm.min.mjs');
    ort.env.wasm.wasmPaths = new URL('./assets/microduck/runtime/onnx/', location.href).href;
    ort.env.wasm.numThreads = 1;
    const errors = {};
    for (const [name, item] of Object.entries(fixture.policies)) {
      const session = await ort.InferenceSession.create(`./assets/microduck/${item.path}`, { executionProviders: ['wasm'] });
      const output = await session.run({ [item.inputName]: new ort.Tensor('float32', Float32Array.from(fixture.input), [1,61]) });
      const values = Array.from(output[session.outputNames[0]].data);
      errors[name] = Math.max(...values.map((value, index) => Math.abs(value - item.output[index])));
      await session.release();
    }
    return errors;
  });
  expect(result.alpha_walking).toBeLessThanOrEqual(1e-5);
  expect(result.roller).toBeLessThanOrEqual(1e-5);
});
