from sweeppy import Sweep

with Sweep('/dev/ttyUSB0') as sweep:
    print(sweep.get_motor_speed())
    print(sweep.get_sample_rate())
    sweep.start_scanning()

    for scan in sweep.get_scans():
        print('{}\n'.format(scan))
