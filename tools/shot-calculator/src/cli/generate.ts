import { writeFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { defaultConfig, generateShotSamples } from '../math/trajectory';
import { fitShotMap } from '../math/polynomial';

const samples = generateShotSamples(defaultConfig);
const fit = fitShotMap(samples);

const javaSource = `package frc.robot.constants;

public final class GeneratedShotMapCoefficients {
    public static final double MIN_DISTANCE_METERS = ${defaultConfig.minDistanceMeters};
    public static final double MAX_DISTANCE_METERS = ${defaultConfig.maxDistanceMeters};
    public static final double MIN_RADIAL_VELOCITY_MPS = ${defaultConfig.minRadialVelocityMps};
    public static final double MAX_RADIAL_VELOCITY_MPS = ${defaultConfig.maxRadialVelocityMps};

    public static final double HOOD_FIT_MAX_ERROR_ROT = ${fit.maxHoodResidualRot.toFixed(6)};
    public static final double ROLLER_FIT_MAX_ERROR_RPS = ${fit.maxRollerResidualRps.toFixed(6)};
    public static final double MIN_ROBUSTNESS_MARGIN = 0.12;

    public static final double[] HOOD_POSITION_COEFFICIENTS = { ${fit.hoodCoefficients.map(format).join(', ')} };
    public static final double[] ROLLER_RPS_COEFFICIENTS = { ${fit.rollerCoefficients.map(format).join(', ')} };

    private GeneratedShotMapCoefficients() {}

    public static double evaluateHoodPositionRot(double distanceMeters, double radialVelocityMps) {
        return evaluate(HOOD_POSITION_COEFFICIENTS, distanceMeters, radialVelocityMps);
    }

    public static double evaluateRollerRps(double distanceMeters, double radialVelocityMps) {
        return evaluate(ROLLER_RPS_COEFFICIENTS, distanceMeters, radialVelocityMps);
    }

    public static double estimateRobustnessMargin(double distanceMeters, double radialVelocityMps) {
        double normalizedRadialVelocity = Math.abs(radialVelocityMps)
            / Math.max(Math.abs(MIN_RADIAL_VELOCITY_MPS), Math.abs(MAX_RADIAL_VELOCITY_MPS));
        return Math.max(0.0, 0.35 - (0.08 * normalizedRadialVelocity));
    }

    public static boolean isInsideDomain(double distanceMeters, double radialVelocityMps) {
        return distanceMeters >= MIN_DISTANCE_METERS
            && distanceMeters <= MAX_DISTANCE_METERS
            && radialVelocityMps >= MIN_RADIAL_VELOCITY_MPS
            && radialVelocityMps <= MAX_RADIAL_VELOCITY_MPS;
    }

    private static double evaluate(double[] coefficients, double distanceMeters, double radialVelocityMps) {
        return coefficients[0]
            + (coefficients[1] * distanceMeters)
            + (coefficients[2] * radialVelocityMps)
            + (coefficients[3] * distanceMeters * distanceMeters)
            + (coefficients[4] * distanceMeters * radialVelocityMps)
            + (coefficients[5] * radialVelocityMps * radialVelocityMps);
    }
}
`;

const outputPath = resolve(process.cwd(), '../../src/main/java/frc/robot/constants/GeneratedShotMapCoefficients.java');
const tsOutputPath = resolve(process.cwd(), 'src/math/generatedCoefficients.ts');
writeFileSync(outputPath, javaSource);
writeFileSync(tsOutputPath, `import type { Polynomial2 } from './types';

export const generatedCoefficients = {
  hood: [
    ${fit.hoodCoefficients.map(format).join(',\n    ')},
  ] satisfies Polynomial2,
  roller: [
    ${fit.rollerCoefficients.map(format).join(',\n    ')},
  ] satisfies Polynomial2,
};
`);
console.log(`Generated ${outputPath} and ${tsOutputPath} from ${samples.length} robust shot samples.`);

function format(value: number): string {
  return Number.isFinite(value) ? value.toPrecision(12) : '0.0';
}
