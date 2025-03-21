package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class SimElevatorIOTalonFX extends ElevatorIOTalonFX
{
    private final TalonFXSimState state;
    private final ElevatorSim sim;
    private final ElevatorConstants constants;

    public SimElevatorIOTalonFX(int id, ElevatorConstants constants, Mass mass)
    {
        super(id, constants);
        state = m_motor.getSimState();
        sim = new ElevatorSim(DCMotor.getKrakenX60(1), constants.gearRatio(), mass.in(Kilograms),
                constants.sprocketCircumference().in(Meters) / (2 * Math.PI), 0, constants.upperLimit().in(Meters),
                true, 0);
        this.constants = constants;
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs)
    {
        super.updateInputs(inputs);
        state.setSupplyVoltage(RobotController.getInputVoltage());
        sim.setInputVoltage(constants.inverted() ? -state.getMotorVoltage() : state.getMotorVoltage());
        sim.update(0.02);
        double sprocketRotations = sim.getVelocityMetersPerSecond() / (constants.sprocketCircumference().in(Meters));
        double rotationsPerSecond = sprocketRotations * constants.gearRatio();
        double dRot = rotationsPerSecond * 0.02;
        state.addRotorPosition(constants.inverted() ? -dRot : dRot);
        state.setRotorVelocity(RotationsPerSecond.of(constants.inverted() ? -rotationsPerSecond : rotationsPerSecond));
    }
}
