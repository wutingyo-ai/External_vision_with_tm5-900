def count_s_v(s_init, v_init, a, dt, t) :
    s_now = s_init
    v_now = v_init
    for i in range(1,int(t/dt)+1) :
        s_now = s_now + v_now*dt + 0.5*a*dt**2
        v_now = v_now + a*dt
        
        print("i:",i,"s_now:",s_now,"v_now:",v_now)
        
    



s0=-35
v0=20
a=-20
dt=0.2
t=1
s_now=s0
count_s_v(s0, v0, a, dt, t)



