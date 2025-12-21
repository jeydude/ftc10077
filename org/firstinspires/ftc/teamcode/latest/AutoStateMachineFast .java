private void startEncoderMove(int flTicks, int frTicks, int blTicks, int brTicks, double power) {

    flStart = fl.getCurrentPosition();
    frStart = fr.getCurrentPosition();
    blStart = bl.getCurrentPosition();
    brStart = br.getCurrentPosition();

    flTarget = flStart + flTicks;
    frTarget = frStart + frTicks;
    blTarget = blStart + blTicks;
    brTarget = brStart + brTicks;

    fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    fl.setPower(power);
    fr.setPower(power);
    bl.setPower(power);
    br.setPower(power);
}
