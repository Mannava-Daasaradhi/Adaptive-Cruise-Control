function a0 = lead_accel(t, amp, w, t0, dur)
%LEAD_ACCEL  Lead-vehicle acceleration maneuver of Sec. IV-A.
%
%   a0 = MA2025.LEAD_ACCEL(t)                     % paper defaults
%   a0 = MA2025.LEAD_ACCEL(t, amp, w, t0, dur)
%
%       a0(t) = 0.5 sin(0.1 (t - 10)),  10 < t < 10 + 20 pi s
%             = 0,                      otherwise
%
%   The window is exactly ONE period of the 0.1 rad/s sine (T = 2 pi/0.1 =
%   20 pi s), so the maneuver starts and ends with a0 = 0 and continuously:
%   no impulsive content is injected, which matters because the platoon is
%   certified only in an L2 / frequency-domain sense.
%
%   Note the velocity excursion this implies:  v0(t) = v_ss + (amp/w)
%   (1 - cos(w (t - t0))) rises to v_ss + 2*amp/w = 25 + 10 = 35 m/s at the
%   half-period and returns to 25 m/s.  It is NOT a small oscillation about
%   the cruise speed -- the desired gap d + hw*v swings by hw*10 = 9.5 m,
%   which is why the spacing errors reach O(1 m) even though the platoon is
%   string stable.
%
%   Vectorised in t.

arguments
    t   (1,:) double
    amp (1,1) double = 0.5
    w   (1,1) double = 0.1
    t0  (1,1) double = 10
    dur (1,1) double = 20*pi
end

inWindow = (t > t0) & (t < t0 + dur);
a0 = zeros(size(t));
a0(inWindow) = amp * sin(w * (t(inWindow) - t0));
end
