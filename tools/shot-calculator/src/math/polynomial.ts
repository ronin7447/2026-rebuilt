import type { Polynomial2, ShotInput, ShotMapFit, TrajectoryCandidate } from './types';

export function evaluatePolynomial(coefficients: Polynomial2, input: ShotInput): number {
  const d = input.distanceMeters;
  const r = input.radialVelocityMps;
  return coefficients[0]
    + coefficients[1] * d
    + coefficients[2] * r
    + coefficients[3] * d * d
    + coefficients[4] * d * r
    + coefficients[5] * r * r;
}

export function fitShotMap(samples: TrajectoryCandidate[]): ShotMapFit {
  const hoodCoefficients = fitPolynomial(samples, (sample) => sample.hoodRot);
  const rollerCoefficients = fitPolynomial(samples, (sample) => sample.rollerRps);
  let maxHoodResidualRot = 0;
  let maxRollerResidualRps = 0;

  for (const sample of samples) {
    maxHoodResidualRot = Math.max(
      maxHoodResidualRot,
      Math.abs(sample.hoodRot - evaluatePolynomial(hoodCoefficients, sample.input)),
    );
    maxRollerResidualRps = Math.max(
      maxRollerResidualRps,
      Math.abs(sample.rollerRps - evaluatePolynomial(rollerCoefficients, sample.input)),
    );
  }

  return {
    hoodCoefficients,
    rollerCoefficients,
    maxHoodResidualRot,
    maxRollerResidualRps,
    samples,
  };
}

function fitPolynomial(samples: TrajectoryCandidate[], output: (sample: TrajectoryCandidate) => number): Polynomial2 {
  const normal = Array.from({ length: 6 }, () => Array.from({ length: 6 }, () => 0));
  const rhs = Array.from({ length: 6 }, () => 0);

  for (const sample of samples) {
    const basis = basisFor(sample.input);
    const y = output(sample);
    for (let row = 0; row < 6; row += 1) {
      rhs[row] += basis[row] * y;
      for (let col = 0; col < 6; col += 1) {
        normal[row][col] += basis[row] * basis[col];
      }
    }
  }

  return solveLinearSystem(normal, rhs) as Polynomial2;
}

function basisFor(input: ShotInput): Polynomial2 {
  const d = input.distanceMeters;
  const r = input.radialVelocityMps;
  return [1, d, r, d * d, d * r, r * r];
}

function solveLinearSystem(matrix: number[][], rhs: number[]): number[] {
  const n = rhs.length;
  const augmented = matrix.map((row, index) => [...row, rhs[index]]);

  for (let pivot = 0; pivot < n; pivot += 1) {
    let bestRow = pivot;
    for (let row = pivot + 1; row < n; row += 1) {
      if (Math.abs(augmented[row][pivot]) > Math.abs(augmented[bestRow][pivot])) {
        bestRow = row;
      }
    }
    [augmented[pivot], augmented[bestRow]] = [augmented[bestRow], augmented[pivot]];
    const pivotValue = augmented[pivot][pivot];
    if (Math.abs(pivotValue) < 1e-12) {
      throw new Error('Shot-map polynomial fit is singular');
    }
    for (let col = pivot; col <= n; col += 1) {
      augmented[pivot][col] /= pivotValue;
    }
    for (let row = 0; row < n; row += 1) {
      if (row === pivot) continue;
      const scale = augmented[row][pivot];
      for (let col = pivot; col <= n; col += 1) {
        augmented[row][col] -= scale * augmented[pivot][col];
      }
    }
  }

  return augmented.map((row) => row[n]);
}
