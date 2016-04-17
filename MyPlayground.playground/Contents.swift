import Cocoa
import simd


var runtime = 100.0 // "seconds"
var k = 1.0

// define state matrix
let A  = double2x2.init([double2.init(x: 1.01, y: 0.03046), double2.init(x: 0.03046, y: 1.021)])
//define input matrix
let B = double2x2.init([double2.init(x: 0.01005, y: 0.0001515), double2.init(x: 0.0001515, y: 0.0101)])
//state vector
var X = double2.init(x:0 , y: 0) //set X0
// setpoint vector
var R = double2.init(x:1.0, y:1.0)
//state feedback gain
let K = double2x2.init([double2.init(x: 151.0087, y: -48.7442), double2.init(x: 50.5154, y: 149.7700)])
// define an integrator
var integrator = double2.init(x:0 , y: 0)
// integrator gain
let Ki = double2x2.init([double2.init(x: 20, y: 0.0), double2.init(x: 0, y: 20)])

//MIMO state feedback + integral control
while k < runtime {
    
    // X = A*X + B*(R - (K*X)) // no integrator control
    integrator += Ki * (R - X)
    X = A * X + B * (integrator - (K * X))
    X.x //plot x1
    X.y //plot x2
    k+=1
}

k = 1;
var sum = 0.0
var derror = 0.0
var ts = 100.0
var timer = 0.0
var pv = 0.0
var error = 0.0
var sp = 1.0
var perror = 0.0
var co = 0.0
var ti = 10.0
var kp = 0.01
var td = 1.0
var time:[Double] = []
var cinput:[Double] = []
var response:[Double] = []

// define a first order process y[k+1] = x0*a^k*u[k]
var x = 0.0
var x0 = 0.0 // intial condition
var a = 0.7
var u:[Double] = [0.0]

//SISO PID control
while k < runtime {
    timer+=1;
    if timer > ts {
        //simulate our process
        var f = 0.0
        for i in 0...(Int(k)-1){
            //compute forced response
            f += pow(a, Double(i)) * u[(Int(k) - 1) - i]
        }
        //compute natural + forced response
        x = x0 * pow(a, k) + f
        pv = x
       
        //compute new controller output
        error = sp - pv
        sum += error
        derror = (derror - error)
        co = kp*(error)
        co+=kp*((ts / ti) * sum)
        co+=kp*((td * derror)/ts)
        perror = error
        
        timer = 0
        time.append(k)
        k+=1
        u.append(co)
    }
}


