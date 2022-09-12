//package org.firstinspires.ftc.teamcode.commands;
//
//import org.firstinspires.ftc.teamcode.shplib.commands.Command;
//import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
//
//public class MoveArmCommand extends Command {
//    private final ArmSubsystem arm;
//    private final double position;
//
//    public MoveArmCommand(ArmSubsystem arm, DriveSubsystem drive, double position) {
//        this.arm = arm;
//        this.position = position;
////        CommandScheduler.getInstance().addCommand(this);
//    }
//
//    @Override
//    public void execute() {
//        drive.driveTo(100, 100);
//        arm.setLifterPosition(position);
//    }
//
//    @Override
//    public boolean isFinished() {
//        return arm.atLifterPosition();
//    }
//
//    @Override
//    public void end() {
//        arm.lifter.setPower(0.0);
//    }
//}
