def circleMotionRV(R, V):
    # Known: R, V, d_mid
    # Find: vl, vr

    # solving for vr fisrt
    vr = (V * (R - WHEEL_D_MID)) / R
    # sub vr to get vl
    vl = 2 * V - vr

    setSpeedsRPS(vl, vr)