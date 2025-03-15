package frc.robot.subsystems;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.nio.ByteBuffer;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

interface Packable {
    static final int kSizeBool = 1;
    static final int kSizeInt8 = 1;
    static final int kSizeInt16 = 2;
    static final int kSizeInt32 = 4;
    static final int kSizeInt64 = 8;
    static final int kSizeFloat = 4;
    static final int kSizeDouble = 8;

    public void pack(ByteBuffer bb);
    public int getSize();
}

record MyRecord(boolean fieldCentric, double xVel, double yVel, double aVel) implements Packable {
    public static final int kSize = kSizeBool + (3 * kSizeDouble);

    public static MyRecord fromSwerveRequest(SwerveRequest.FieldCentric fc) {
        return new MyRecord(true, fc.VelocityX, fc.VelocityY, fc.RotationalRate);
    }
    public static MyRecord fromSwerveRequest(SwerveRequest.RobotCentric rc) {
        return new MyRecord(false, rc.VelocityX, rc.VelocityY, rc.RotationalRate);
    }
    public static MyRecord fromSwerveRequest(SwerveRequest sr) {
        if (sr instanceof SwerveRequest.FieldCentric)
            return fromSwerveRequest((SwerveRequest.FieldCentric)sr);
        else if (sr instanceof SwerveRequest.RobotCentric)
            return fromSwerveRequest((SwerveRequest.RobotCentric)sr);
        else {
            System.out.println("MyRecord: sr is neither FieldCentric nor RobotCentric");
            return null;
        }
    }
    public static MyRecord unpack(ByteBuffer bb) {
        boolean fieldCentric = bb.get() == 1;
        double xVel = bb.getDouble();
        double yVel = bb.getDouble();
        double aVel = bb.getDouble();

        return new MyRecord(fieldCentric, xVel, yVel, aVel);
    }

    @Override
    public void pack(ByteBuffer bb) {
        bb.put(fieldCentric ? (byte)1 : (byte)0);
        bb.putDouble(xVel);
        bb.putDouble(yVel);
        bb.putDouble(aVel);
    }
    @Override
    public int getSize() {
        return kSize;
    }

    public SwerveRequest makeSwerveRequest(SwerveRequest.FieldCentric fc, SwerveRequest.RobotCentric rc) {
        if (fieldCentric) {
            return fc.withVelocityX(xVel).withVelocityY(yVel).withRotationalRate(aVel);
        } else {
            return rc.withVelocityX(xVel).withVelocityY(yVel).withRotationalRate(aVel);
        }
    }
}

class ReplayerCommand extends Command {
    private final SwerveRequest.FieldCentric fieldCentric;
    private final SwerveRequest.RobotCentric robotCentric;
    private final String filename;
    private final ByteBuffer bb = ByteBuffer.allocateDirect(MyRecord.kSize);
    private boolean finished = false;
    private FileInputStream readStream;

    public ReplayerCommand(SwerveRequest.FieldCentric fieldCentric, SwerveRequest.RobotCentric robotCentric, String filename) {
        addRequirements(RobotContainer.drivetrain);

        this.fieldCentric = fieldCentric;
        this.robotCentric = robotCentric;
        this.filename = filename;
    }

    @Override
    public void initialize() {
        finished = false;

        try {
            readStream = new FileInputStream(filename);
        } catch (Exception e) {
            System.out.println("ReplayerCommand error: " + e);
        }
    }

    @Override
    public void execute() {
        byte[] arr;
        try {
            arr = readStream.readNBytes(MyRecord.kSize);

            if (arr.length < MyRecord.kSize) {
                finished = true;
                return;
            }

            bb.clear();
            bb.put(arr);
        } catch (Exception e) {
            System.out.println("ReplayerCommand error: " + e);
        }

        MyRecord rec = MyRecord.unpack(bb);
        SwerveRequest req = rec.makeSwerveRequest(fieldCentric, robotCentric);
        RobotContainer.drivetrain.setControl(req);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        try {
            readStream.close();
        } catch (Exception e) {
            System.out.println("ReplayerCommand error: " + e);
        }
    }
}

public class Recorder extends SubsystemBase {
    private boolean recording = false;
    private int numFilesWritten = 0;
    private FileOutputStream writeStream;
    private SwerveRequest robotDrive = null;
    private ByteBuffer bb = ByteBuffer.allocateDirect(MyRecord.kSize);

    public Recorder() {
    }

    public static String makeFilename(int fileNumber) {
        return "record-" + fileNumber + ".txt";
    }

    public boolean isRecording() {
        return recording;
    }

    public void setRobotDrive(SwerveRequest drive) {
        robotDrive = drive;
    }

    public void startRecording() {
        if (recording) {
            return;
        }

        recording = true;

        try {
            String filename = makeFilename(numFilesWritten);
            writeStream = new FileOutputStream(filename);
        } catch (Exception e) {
            System.out.println("Recorder error: " + e);
            return;
        }
    }

    @Override
    public void periodic() {
        if (recording) {
            if (robotDrive == null) {
                System.out.println("Recorder: robotDrive is null");
            }

            MyRecord rec = MyRecord.fromSwerveRequest(robotDrive);

            bb.clear();
            rec.pack(bb);
            try {
                writeStream.write(bb.array());
            } catch (Exception e) {
                System.out.println("Recorder error: " + e);
            }
        }
    }

    public void stopRecording() {
        if (!recording) {
            return;
        }

        recording = false;
        numFilesWritten++;

        try {
            writeStream.close();
        } catch (Exception e) {
            System.out.println("Recorder error: " + e);
        }
    }

    public Command replayCommand(SwerveRequest.FieldCentric fc, SwerveRequest.RobotCentric rc, int fileNumber) {
        if (recording) {
            return null;
        }
        if (fileNumber >= (numFilesWritten)) {
            System.out.println(
                "Recorder: fileNumber (" + fileNumber + ") is less than numFilesWritten (" + numFilesWritten + ")"
            );
            return null;
        }
        return new ReplayerCommand(fc, rc, makeFilename(fileNumber));
    }
}
