import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline


class SupportFilesCar:
    ''' The following functions interact with the main file'''

    def __init__(self):
        ''' Load the constants that do not change'''

        # Constants
        g=9.81
        m=1500
        Iz=3000
        Cf=38000
        Cr=66000
        lf=2
        lr=3
        Ts=0.02
        mju=0.02 # friction coefficient

        ####################### Lateral control #################################

        outputs=4 # number of outputs
        inputs=2 # number of inputs
        hz = 10 # horizon period

        trajectory=4 # Choose 1, 2 or 3, nothing else
        version=1 # This is only for trajectory 3 (Choose 1 or 2)

        # Matrix weights for the cost function (They must be diagonal)

        if trajectory==3 and version==2:  #3rd trajectoris with larger distances
            # Weights for trajectory 3, version 2
            Q=np.matrix('100 0 0 0;0 20000 0 0;0 0 1000 0;0 0 0 1000') # weights for outputs (all samples, except the last one)
            S=np.matrix('100 0 0 0;0 20000 0 0;0 0 1000 0;0 0 0 1000') # weights for the final horizon period outputs
            R=np.matrix('100 0;0 1') # weights for inputs
        elif trajectory==3:
            # Weights for trajectory 3, version 1
            Q=np.matrix('100 0 0 0;0 20000 0 0;0 0 1000 0;0 0 0 1000') # weights for outputs (all samples, except the last one)
            S=np.matrix('100 0 0 0;0 20000 0 0;0 0 1000 0;0 0 0 1000') # weights for the final horizon period outputs
            R=np.matrix('100 0;0 1') # weights for inputs
        else:
            # Weights for trajectories 1 & 2
            Q=np.matrix('1 0 0 0;0 200 0 0;0 0 50 0;0 0 0 50') # weights for outputs (all samples, except the last one)
            S=np.matrix('1 0 0 0;0 200 0 0;0 0 50 0;0 0 0 50') # weights for the final horizon period outputs
            R=np.matrix('100 0;0 1') # weights for inputs

        # Please do not modify the time_length!
        delay=0
        if trajectory==1:
            time_length = 60. # simultion length
            x_lim=1000   # x and y limits
            y_lim=1000
        elif trajectory ==2:
            time_length = 140.
            x_lim=1000
            y_lim=1000
        elif trajectory == 3:
            if version==1:
                x_lim=170
                y_lim=160
            else:
                x_lim=170*version
                y_lim=160*version

            # since trajectry 3 is made up of 11 different trajectories we define time length for 1st and other trajectoris
            first_section=10  # 14 sec for first part of the trajectory
            other_sections=11  # 14 sec for other parts of the trajectories
            time_length=first_section+other_sections*8  # total time length of 11 trajectories
            delay=np.zeros(10)  

            # this loop defines the time start and end of each trajectories
            for dly in range(1,len(delay)):
                delay[dly]=first_section+(dly-1)*other_sections

            #uncomment below to see the time spans
            # print(delay)
            # exit()  
        elif trajectory == 4:
            time_length = 140
            x_lim = 170*2
            y_lim = 160*2
        else:
            print("trajectory: 1,2 or 3; version: 1 or 2")

        self.constants={'g':g,'m':m,'Iz':Iz,'Cf':Cf,'Cr':Cr,'lf':lf,'lr':lr,\
        'Ts':Ts,'mju':mju,'Q':Q,'S':S,'R':R,'outputs':outputs,'inputs':inputs,\
        'hz':hz,'delay':delay,'time_length':time_length,'trajectory':trajectory,\
        'version':version,'x_lim':x_lim,'y_lim':y_lim}
        # exit()
        return None
    
    def generate_trajectory(self,waypoints):
        # Unpack latitude and longitude from waypoints
        lat, lon = zip(*waypoints)
        cs_lat = CubicSpline(range(len(lat)), lat)  # Cubic spline interpolation for latitude
        cs_lon = CubicSpline(range(len(lon)), lon)  # Cubic spline interpolation for longitude

        # Generate a denser trajectory for smoother transitions
        spline_points = np.linspace(0, len(lat) - 1, num=len(lat) *700) #600 ,30
        lat_dense = cs_lat(spline_points)
        lon_dense = cs_lon(spline_points)

        # Calculate yaw angles (cyaw) and curvature (ck) for the dense trajectory
        cyaw = np.arctan2(np.gradient(lon_dense), np.gradient(lat_dense))
        ck = np.gradient(np.gradient(lon_dense, spline_points), spline_points) / (1 + (np.gradient(lat_dense, spline_points))**2)**1.5

        return lat_dense, lon_dense, cyaw, ck
    
    def gps_to_xy(self, gps_coords, start_x=4, start_y=4):
        # Your GPS to XY conversion logic goes here
        # This can involve various transformations depending on the coordinate system and units used
        
        # For demonstration purposes, let's assume a simple conversion:
        # Here, we'll just subtract the first GPS coordinates to make them relative
        ref_gps = gps_coords[0]
        x_coords = [coord[0] - ref_gps[0] + start_x for coord in gps_coords]
        y_coords = [coord[1] - ref_gps[1] + start_y for coord in gps_coords]
        
        return x_coords, y_coords


    def trajectory_generator(self,t):
        '''This method creates the trajectory for a car to follow'''

        Ts=self.constants['Ts']
        trajectory=self.constants['trajectory']
        x_lim=self.constants['x_lim']
        y_lim=self.constants['y_lim']

        # Define trajectories
        if trajectory==1:
            X=15*t
            Y=750/900**2*X**2+250

            # # Plot the world
            # plt.plot(X,Y,'b',linewidth=2,label='The trajectory')
            # plt.xlabel('X-position [m]',fontsize=15)
            # plt.ylabel('Y-position [m]',fontsize=15)
            # plt.grid(True)
            # plt.legend(loc='upper right',fontsize='small')
            # plt.xlim(0,x_lim)
            # plt.ylim(0,y_lim)
            # plt.xticks(np.arange(0,x_lim+1,int(x_lim/10)))
            # plt.yticks(np.arange(0,y_lim+1,int(y_lim/10)))
            # plt.show()
            #
            # plt.plot(t,X,'b',linewidth=2,label='X ref')
            # plt.plot(t,Y,'r',linewidth=2,label='Y ref')
            # plt.xlabel('t-position [s]',fontsize=15)
            # plt.ylabel('X,Y-position [m]',fontsize=15)
            # plt.grid(True)
            # plt.legend(loc='upper right',fontsize='small')
            # plt.xlim(0,t[-1])
            # plt.show()
            # # exit()

        elif trajectory==2:  #trajectory 2 consist of 3 sub trajectories

            X1=15*t[0:int(40/Ts+1)]
            Y1=50*np.sin(2*np.pi*0.75/40*t[0:int(40/Ts+1)])+250

            X2=300*np.cos(2*np.pi*0.5/60*(t[int(40/Ts+1):int(100/Ts+1)]-40)-np.pi/2)+600
            Y2=300*np.sin(2*np.pi*0.5/60*(t[int(40/Ts+1):int(100/Ts+1)]-40)-np.pi/2)+500

            X3=600-15*(t[int(100/Ts+1):int(140/Ts+1)]-100)
            Y3=50*np.cos(2*np.pi*0.75/40*(t[int(100/Ts+1):int(140/Ts+1)]-100))+750

            # here we are not using cubic polynomial to merge the trajectories, instead we jut concatinate them

            X=np.concatenate((X1,X2),axis=0)
            Y=np.concatenate((Y1,Y2),axis=0)

            X=np.concatenate((X,X3),axis=0)
            Y=np.concatenate((Y,Y3),axis=0)

            # # Plot the world
            # plt.plot(X,Y,'b',linewidth=2,label='The trajectory')
            # plt.xlabel('X-position [m]',fontsize=15)
            # plt.ylabel('Y-position [m]',fontsize=15)
            # plt.xlim(0,x_lim)
            # plt.ylim(0,y_lim)
            # plt.xticks(np.arange(0,x_lim+1,int(x_lim/10)))
            # plt.yticks(np.arange(0,y_lim+1,int(y_lim/10)))
            # plt.grid(True)
            # plt.legend(loc='upper right',fontsize='small')
            # plt.show()
            #
            # plt.plot(t,X,'b',linewidth=2,label='X ref')
            # plt.plot(t,Y,'r',linewidth=2,label='Y ref')
            # plt.xlabel('t-position [s]',fontsize=15)
            # plt.ylabel('X,Y-position [m]',fontsize=15)
            # plt.grid(True)
            # plt.legend(loc='upper right',fontsize='small')
            # plt.xlim(0,t[-1])
            # plt.show()
            # # exit()
        
        elif trajectory ==4:
            waypoints = [[195034.87903749224,1948396.012324538],
            [195034.87674161655,1948396.258831263],
            [195034.869355784,1948396.645694745],
            [195034.86376979452,1948397.0037049472],
            [195034.8531896953,1948397.5035386342],
            [195034.84197458043,1948397.9425011862],
            [195034.82759256795,1948398.5410917036],
            [195034.81363620283,1948399.0606673467],
            [195034.79604800022,1948399.616677373],
            [195034.7782866147,1948400.3650969919],
            [195034.76044840034,1948401.0085068673],
            [195034.73818779082,1948401.8650640745],
            [195034.71242332747,1948402.5896145795],
            [195034.68061090485,1948403.5430691852],
            [195034.65558138947,1948404.5488263934],
            [195034.6213242652,1948405.4251174484],
            [195034.59438105093,1948406.318291414],
            [195034.58582753525,1948407.4755657031],
            [195034.5538216011,1948408.6941630135],
            [195034.5408874482,1948409.7154049159],
            [195034.4903843848,1948411.031137472],
            [195034.4731204997,1948412.1159038786],
            [195034.44897528074,1948413.5184061509],
            [195034.43223824943,1948414.6695170917],
            [195034.43009625765,1948416.143538826],
            [195034.41359901434,1948417.352804506],
            [195034.40256515704,1948418.911565816],
            [195034.39788935674,1948420.1766889447],
            [195034.3843325712,1948421.7822484912],
            [195034.34287977128,1948423.0905762957],
            [195034.3312745588,1948424.7518296842],
            [195034.3290935159,1948426.1023181702],
            [195034.3363579364,1948427.8099710278],
            [195034.33795529118,1948429.19659355],
            [195034.35347833982,1948430.956448343],
            [195034.3673241191,1948432.381713539],
            [195034.38616786085,1948434.1812804604],
            [195034.40887792667,1948435.6458722702],
            [195034.43646937195,1948437.4999945692],
            [195034.5083899102,1948439.3926052083],
            [195034.53757107301,1948440.9097496853],
            [195034.5776655841,1948442.808652773],
            [195034.60593516772,1948444.3017858414],
            [195034.63070777492,1948445.7748685395],
            [195034.6297352826,1948447.55166518],
            [195034.6490274472,1948448.976908894],
            [195034.6590254273,1948450.7099151742],
            [195034.69073477306,1948452.4243307568],
            [195034.6827692783,1948453.7735001484],
            [195034.68903420714,1948455.1087301937],
            [195034.6909131585,1948456.750431544],
            [195034.69724983457,1948458.0484037844],
            [195034.69857280963,1948459.6447053163],
            [195034.70044971965,1948460.8966303878],
            [195034.69722467734,1948462.4329731679],
            [195034.69238261864,1948463.9434492302],
            [195034.68359918587,1948465.1358407824],
            [195034.67916166963,1948466.604801079],
            [195034.6725301978,1948467.7628645715],
            [195034.65988998508,1948468.9049679437],
            [195034.65410833052,1948470.3246046815],
            [195034.64506853657,1948471.71781204],
            [195034.63442452598,1948472.816513931],
            [195034.62521761705,1948474.1782245312],
            [195034.61956021955,1948475.2644085006],
            [195034.61883918595,1948476.6286861098],
            [195034.60965570135,1948477.7298940143],
            [195034.61383986927,1948479.1367476408],
            [195034.60693079515,1948480.2689954813],
            [195034.6023679989,1948481.6972767522],
            [195034.59356504062,1948482.854490501],
            [195034.58898033184,1948484.0143756752],
            [195034.58531165955,1948485.4765305368],
            [195034.5796969664,1948486.9444518867],
            [195034.5677696187,1948488.1218625607],
            [195034.55763688235,1948489.60151638],
            [195034.5453037109,1948490.791861991],
            [195034.54730815472,1948492.277609338],
            [195034.54401174537,1948493.474803423],
            [195034.54406938129,1948494.9755481167],
            [195034.5467068076,1948496.176844352],
            [195034.55217412047,1948497.6883874],
            [195034.5519330133,1948498.9050634964],
            [195034.56472531945,1948500.4334983611],
            [195034.57165706414,1948501.9708951502],
            [195034.58438202908,1948503.2037984917],
            [195034.5923112541,1948504.447506261],
            [195034.60547736098,1948506.0027383326],
            [195034.61920033736,1948507.2564103536],
            [195034.6378245314,1948508.8265052678],
            [195034.6558108549,1948510.0858872323],
            [195034.68427952382,1948511.6639972238],
            [195034.7103203763,1948513.2407747302],
            [195034.7372227101,1948514.5052860854],
            [195034.76036281278,1948516.0896024297],
            [195034.77728039335,1948517.3582037871],
            [195034.80049394656,1948518.9470325613],
            [195034.81691260304,1948520.2258215907],
            [195034.84138538307,1948521.8275287594],
            [195034.868946398,1948523.116343218],
            [195034.8975652435,1948524.727535039],
            [195034.9271135499,1948526.0104265646],
            [195034.96124386066,1948527.6087048203],
            [195035.00803055486,1948528.8716846209],
            [195035.08914324807,1948530.439044312],
            [195035.19201541247,1948531.6125025614],
            [195035.39428723435,1948532.9638169925],
            [195035.61430011288,1948533.9366627831],
            [195035.97776237837,1948535.004886556],
            [195036.33542527337,1948535.763532074],
            [195036.87274475655,1948536.6227769493],
            [195037.37871456833,1948537.2476823048],
            [195038.0979452337,1948537.9974942445],
            [195038.7383211284,1948538.544448738],
            [195039.6193988197,1948539.1978252814],
            [195040.3642465541,1948539.7066846644],
            [195041.29179818617,1948540.3443970021],
            [195042.04888113332,1948540.863814569],
            [195043.00901203149,1948541.5060775417],
            [195043.78513567342,1948542.0093783608],
            [195044.74560787238,1948542.6443943395],
            [195045.52877904093,1948543.1400870217],
            [195046.51179332432,1948543.7672191667],
            [195047.3047173712,1948544.2538332625],
            [195048.2903835096,1948544.8659965862],
            [195049.08868049833,1948545.3452673429],
            [195050.08888061307,1948545.9417106372],
            [195050.8997724364,1948546.3977598138],
            [195051.9156387764,1948546.9562705115],
            [195052.7468444805,1948547.3776199033],
            [195053.80260192996,1948547.8855006092],
            [195054.67277485167,1948548.2814366142],
            [195055.79220336842,1948548.759119625],
            [195056.71599930426,1948549.1229722325],
            [195057.91592588526,1948549.5488646447],
            [195059.15637733595,1948549.9389724894],
            [195060.18287745895,1948550.2113969729],
            [195061.50496687414,1948550.4964833602],
            [195062.58866321953,1948550.6667831768],
            [195063.6893852929,1948550.785505108],
            [195065.0705712475,1948550.8417435694],
            [195066.18194376616,1948550.8427544136],
            [195067.5646264415,1948550.7619967053],
            [195068.6732611784,1948550.6333309056],
            [195070.0556351416,1948550.4009765768],
            [195071.15334953586,1948550.1513004934],
            [195072.52436084795,1948549.7826517364],
            [195073.8851536388,1948549.328108498],
            [195074.96145143747,1948548.9261258],
            [195076.01917835415,1948548.4753109848],
            [195077.3191811847,1948547.849362125],
            [195078.6073058152,1948547.1424423845],
            [195079.62725309172,1948546.5446691653],
            [195080.62835609494,1948545.8966050784],
            [195081.86792163373,1948545.0305363971],
            [195083.104085448,1948544.12718764],
            [195084.07983641795,1948543.3628006275],
            [195085.0589886115,1948542.5717205403],
            [195086.26206157927,1948541.5478908755],
            [195087.43133545129,1948540.5001019929],
            [195088.30821029656,1948539.6414473366],
            [195089.27824354867,1948538.5686551076],
            [195089.95245141507,1948537.7319321733],
            [195090.63901512322,1948536.6932013761],
            [195091.06436208508,1948535.8766578373],
            [195091.43928503813,1948534.882812529],
            [195091.64931150552,1948534.1294887143],
            [195091.81741664282,1948533.2498823644],
            [195091.8945146178,1948532.565534867],
            [195091.94570368828,1948531.8772476749],
            [195091.9817764161,1948530.9839813802],
            [195091.98845570142,1948530.0388917516],
            [195091.995278827,1948529.2499207358],
            [195091.99201572305,1948528.22379383],
            [195091.98538146334,1948527.3717572116],
            [195091.97136127818,1948526.2647513412],
            [195091.9659754192,1948525.356682665],
            [195091.93504761273,1948524.199807216],
            [195091.89558548827,1948523.240911677],
            [195091.7987654071,1948522.014636507],
            [195091.71204872217,1948521.0145573486],
            [195091.57928262913,1948519.7269331445],
            [195091.47183542504,1948518.6660429419],
            [195091.33713696778,1948517.3085304468],
            [195091.18760983186,1948515.9165117992],
            [195091.07927402726,1948514.7822250058],
            [195090.92521204866,1948513.3487698466],
            [195090.8031618303,1948512.1803856466],
            [195090.67857022572,1948510.9969533915],
            [195090.51777665108,1948509.4981659683],
            [195090.38381856238,1948508.280568608],
            [195090.22140115764,1948506.7372232645],
            [195090.08687863947,1948505.4842964876],
            [195089.91872012214,1948503.8884097144],
            [195089.78545908412,1948502.5860655105],
            [195089.61994810228,1948500.927793035],
            [195089.47312681066,1948499.5866327183],
            [195089.2976024281,1948497.8801041902],
            [195089.1623397995,1948496.4936946032],
            [195088.9812977072,1948494.738944671],
            [195088.81759921063,1948492.955991905],
            [195088.67399503457,1948491.5089128464],
            [195088.5562556604,1948490.0571716859],
            [195088.40236767835,1948488.2418882283],
            [195088.28187339747,1948486.7955912754],
            [195088.13044912845,1948484.9969785314],
            [195087.95274404497,1948483.2156626822],
            [195087.82229244828,1948481.7943971334],
            [195087.6773384208,1948480.383136487],
            [195087.48347283353,1948478.6304156801],
            [195087.31014396172,1948477.2417638227],
            [195087.07940278752,1948475.564000099],
            [195086.85095906444,1948474.3062727011],
            [195086.5224475213,1948472.853154372],
            [195086.21506213784,1948471.7934158691],
            [195085.75239517228,1948470.6167831589],
            [195085.3013768156,1948469.7984874658],
            [195084.67015192498,1948468.8842730138],
            [195084.0536005119,1948468.2810823792],
            [195083.22544967072,1948467.621021206],
            [195082.30567722037,1948467.122476691],
            [195081.54584162694,1948466.8049754668],
            [195080.55840538137,1948466.5389564184],
            [195079.75848998135,1948466.4356595343],
            [195078.76403976986,1948466.398341978],
            [195077.96409043018,1948466.4150977184],
            [195076.98196673428,1948466.4154271793],
            [195076.19484822746,1948466.3936960765],
            [195075.22749233388,1948466.3745300795],
            [195074.45142916997,1948466.357656768],
            [195073.4692074184,1948466.3405357962],
            [195072.6664748913,1948466.3167874839],
            [195071.62965800986,1948466.3048131133],
            [195070.7603400986,1948466.2879376397],
            [195069.62792952242,1948466.2603449097],
            [195068.68646729604,1948466.243481906],
            [195067.47588742853,1948466.2138051873],
            [195066.47916696788,1948466.1858463197],
            [195065.2015718736,1948466.1589276572],
            [195064.68267277532,1948466.1477999347]
            ]
            
            # f_x=np.array([0,80,110,140,160,110,40,10,40,70,110,150]) *2 # x end ponts for sub trajectories
            # f_y=np.array([40,20,20,60,100,140,140,80,60,60,90,90])*2
            # points_list = [[x, y] for x, y in zip(f_x, f_y)]

            f_x=np.array([4,80,135,159,157,142,124,91,75,75])*2 # x end ponts for sub trajectories
            f_y=np.array([4,4,5,27,43,61,60,57,50,40])*2 # y end ponts for sub trajectories

            points_list = [[x, y] for x, y in zip(f_x, f_y)]
            # for ss in waypoints:
            #     ss.reverse()
            # X, Y = self.gps_to_xy(waypoints)
            # points_list = [[x, y] for x, y in zip(X, Y)]
            # xtest=[]
            # ytest=[]
            # for wp in waypoints:
            #     xtest.append(wp[0])
            #     ytest.append(wp[1])
            # print(len(xtest),len(ytest))
            X,Y,yaw,ck = self.generate_trajectory(points_list)
            
            X = np.append(X,X[-1])
            Y = np.append(Y,Y[-1])
            # X = X[:-199]
            # Y = Y[:-199]
            print(len(t))
            print(len(X),len(Y))
            # print(len(X))
            print(X.shape)
            # exit()
            ################
            # Plot the world
            # plt.plot(X,Y,'b',linewidth=2,label='The trajectory')
            # plt.xlabel('X-position [m]',fontsize=15)
            # plt.ylabel('Y-position [m]',fontsize=15)
            # plt.xlim(0,x_lim)
            # plt.ylim(0,y_lim)
            # plt.xticks(np.arange(0,x_lim+1,int(x_lim/10)))
            # plt.yticks(np.arange(0,y_lim+1,int(y_lim/10)))
            # plt.grid(True)
            # plt.legend(loc='upper right',fontsize='small')
            # plt.show()
            
            # plt.plot(t,X,'b',linewidth=2,label='X ref')
            # plt.plot(t,Y,'r',linewidth=2,label='Y ref')
            # plt.xlabel('t-position [s]',fontsize=15)
            # plt.ylabel('X,Y-position [m]',fontsize=15)
            # plt.grid(True)
            # plt.legend(loc='upper right',fontsize='small')
            # plt.xlim(0,t[-1])
            # plt.show()
            # exit()


        else: # this is 3rd trajectories with 11 sub trajectories

            # Attention! Do not let x_dot become 0 m/s. It is in the denominator.
            delay=self.constants['delay']  # delay is an array consist of time end points for different trajectories
            version=self.constants['version']

            # X & Y levels
            # f_x=np.array([0,60,110,140,160,110,40,10,40,70,110,150])*version # x end ponts for sub trajectories
            # f_y=np.array([40,20,20,60,100,140,140,80,60,60,90,90])*version # y end ponts for sub trajectories

            f_x=np.array([4,80,135,159,157,142,124,91,75,75])*version # x end ponts for sub trajectories
            f_y=np.array([4,4,5,27,43,61,60,57,50,40])*version # y end ponts for sub trajectories
            # X & Y derivatives
            # f_x_dot is not X_dot this is just to calculate the trajectory
            f_x_dot=np.array([2,1,1,0,0,-1,-1,-1,0,0])*3*version # x_dot end points, 3 is just to increase the magntude of velocity
            f_y_dot=np.array([0,0,0,1,0,0,0,0,-1,-1])*3*version

            # define empty array for global X and Y dimensions
            X=[]
            Y=[]

            # this loop is to calculate the cubic polynomial coefficients for x and y
            for i in range(0,len(delay)-1):
                # Extract the time elements for each section separately
                if i != len(delay)-2:
                    t_temp=t[int(delay[i]/Ts):int(delay[i+1]/Ts)]
                else:
                    t_temp=t[int(delay[i]/Ts):int(delay[i+1]/Ts+1)]

                # Generate data for a subtrajectory
                M=np.array([[1,t_temp[0],t_temp[0]**2,t_temp[0]**3],\
                            [1,t_temp[-1],t_temp[-1]**2,t_temp[-1]**3],\
                            [0,1,2*t_temp[0],3*t_temp[0]**2],\
                            [0,1,2*t_temp[-1],3*t_temp[-1]**2]])

                c_x=np.array([[f_x[i]],[f_x[i+1]-f_x_dot[i+1]*Ts],[f_x_dot[i]],[f_x_dot[i+1]]])
                c_y=np.array([[f_y[i]],[f_y[i+1]-f_y_dot[i+1]*Ts],[f_y_dot[i]],[f_y_dot[i+1]]])


                a_x=np.matmul(np.linalg.inv(M),c_x)
                a_y=np.matmul(np.linalg.inv(M),c_y)

                # Compute X and Y values
                X_temp=a_x[0][0]+a_x[1][0]*t_temp+a_x[2][0]*t_temp**2+a_x[3][0]*t_temp**3
                Y_temp=a_y[0][0]+a_y[1][0]*t_temp+a_y[2][0]*t_temp**2+a_y[3][0]*t_temp**3

                # Concatenate X and Y values at every iteration
                X=np.concatenate([X,X_temp])
                Y=np.concatenate([Y,Y_temp])

            # Round the numbers to avoid numerical errors
            X=np.round(X,8) # round the values to 8th decimal places
            Y=np.round(Y,8)

            print(len(X),len(Y))
            # # Plot the world
            # plt.subplots_adjust(left=0.05,bottom=0.05,right=0.95,top=0.95,wspace=0.15,hspace=0.2)
            # plt.plot(X,Y,'b',linewidth=2,label='The ref trajectory')
            # plt.xlabel('X-position [m]',fontsize=15)
            # plt.ylabel('Y-position [m]',fontsize=15)
            # plt.grid(True)
            # plt.legend(loc='upper right',fontsize='small')
            # plt.xlim(0,x_lim)
            # plt.ylim(0,y_lim)
            # plt.xticks(np.arange(0,x_lim+1,int(x_lim/10)))
            # plt.yticks(np.arange(0,y_lim+1,int(y_lim/10)))
            # plt.show()
            
            # plt.subplots_adjust(left=0.05,bottom=0.05,right=0.95,top=0.95,wspace=0.15,hspace=0.2)
            # plt.plot(t,X,'b',linewidth=2,label='X ref')
            # plt.plot(t,Y,'r',linewidth=2,label='Y ref')
            # plt.xlabel('t-position [s]',fontsize=15)
            # plt.ylabel('X,Y-position [m]',fontsize=15)
            # plt.grid(True)
            # plt.legend(loc='upper right',fontsize='small')
            # plt.xlim(0,t[-1])
            # plt.show()
            # exit()

        # Vector of x and y changes per sample time
            
        #if X = [a,b,c,d,e] then dx = [b-a,c-b,d-c,e-d] sam is for dy   
        dX=X[1:len(X)]-X[0:len(X)-1]  # can also achieved by dX = np.diff(X)
        dY=Y[1:len(Y)]-Y[0:len(Y)-1]  # or np.diff(Y)

        # calculate X_dot and Y_dot, Ts = 0.02
        X_dot=dX/Ts
        Y_dot=dY/Ts

        # length of X_dot and Y_dot is one less than X and Y, hence to make it of same length we concatinate first value twice
        X_dot=np.concatenate(([X_dot[0]],X_dot),axis=0)
        Y_dot=np.concatenate(([Y_dot[0]],Y_dot),axis=0)

        # Define the reference yaw angles
        psi=np.zeros(len(X))
        psiInt=psi
        psi[0]=np.arctan2(dY[0],dX[0])  # use arctan2 because it takes care of quadrant values
        psi[1:len(psi)]=np.arctan2(dY[0:len(dY)],dX[0:len(dX)])

        # We want the yaw angle to keep track the amount of rotations
        dpsi=psi[1:len(psi)]-psi[0:len(psi)-1]  # can also use np.diff(psi)

        # this loop keeps track of rotation and make psi smooth, returns psiInt
        psiInt[0]=psi[0]
        for i in range(1,len(psiInt)):
            if dpsi[i-1]<-np.pi:
                psiInt[i]=psiInt[i-1]+(dpsi[i-1]+2*np.pi)
            elif dpsi[i-1]>np.pi:
                psiInt[i]=psiInt[i-1]+(dpsi[i-1]-2*np.pi)
            else:
                psiInt[i]=psiInt[i-1]+dpsi[i-1]

        # calculate X_dot and Y_dot in body frame from inertial frame using rotation matrix as discused
        x_dot_body=np.cos(psiInt)*X_dot+np.sin(psiInt)*Y_dot
        y_dot_body=-np.sin(psiInt)*X_dot+np.cos(psiInt)*Y_dot
        y_dot_body=np.round(y_dot_body)  # round to avoid small numerical error

        # # Plot the body frame velocity
        # # plt.plot(t,x_dot_body,'g',linewidth=2,label='x_dot ref')
        # plt.plot(t,X_dot,'b',linewidth=2,label='X_dot ref')
        # plt.plot(t,Y_dot,'r',linewidth=2,label='Y_dot ref')
        # plt.xlabel('t [s]',fontsize=15)
        # plt.ylabel('X_dot_ref, Y_dot_ref [m/s]',fontsize=15)
        # plt.grid(True)
        # plt.legend(loc='upper right',fontsize='small')
        # plt.show()
        #
        # # Plot the reference yaw angle
        # plt.plot(t,psiInt,'g',linewidth=2,label='Psi ref')
        # plt.xlabel('t [s]',fontsize=15)
        # plt.ylabel('Psi_ref [rad]',fontsize=15)
        # plt.grid(True)
        # plt.legend(loc='upper right',fontsize='small')
        # plt.show()
        # exit()
        # print("enddd: ",len(x_dot_body),(len(y_dot_body)),(len(psiInt)),(len(X)),(len(Y)))
        # exit()
        return x_dot_body,y_dot_body,psiInt,X,Y

    def state_space(self,states,delta,a):
        '''This function forms the state space matrices and transforms them in the discrete form'''

        # Get the necessary constants
        g=self.constants['g']
        m=self.constants['m']
        Iz=self.constants['Iz']
        Cf=self.constants['Cf']
        Cr=self.constants['Cr']
        lf=self.constants['lf']
        lr=self.constants['lr']
        Ts=self.constants['Ts']
        mju=self.constants['mju']

        # Get the necessary states
        x_dot=states[0]
        y_dot=states[1]
        psi=states[2]

        # Get the state space matrices for the control
        A11=-mju*g/x_dot
        A12=Cf*np.sin(delta)/(m*x_dot)
        A14=Cf*lf*np.sin(delta)/(m*x_dot)+y_dot
        A22=-(Cr+Cf*np.cos(delta))/(m*x_dot)
        A24=-(Cf*lf*np.cos(delta)-Cr*lr)/(m*x_dot)-x_dot
        A34=1
        A42=-(Cf*lf*np.cos(delta)-lr*Cr)/(Iz*x_dot)
        A44=-(Cf*lf**2*np.cos(delta)+lr**2*Cr)/(Iz*x_dot)
        A51=np.cos(psi)
        A52=-np.sin(psi)
        A61=np.sin(psi)
        A62=np.cos(psi)

        B11=-1/m*np.sin(delta)*Cf
        B12=1
        B21=1/m*np.cos(delta)*Cf
        B41=1/Iz*np.cos(delta)*Cf*lf


        A=np.array([[A11, A12, 0, A14, 0, 0],[0, A22, 0, A24, 0, 0],[0, 0, 0, A34, 0, 0],\
        [0, A42, 0, A44, 0, 0],[A51, A52, 0, 0, 0, 0],[A61, A62, 0, 0, 0, 0]])
        B=np.array([[B11, B12],[B21, 0],[0, 0],[B41, 0],[0, 0],[0, 0]])
        C=np.array([[1, 0, 0, 0, 0, 0],[0, 0, 1, 0, 0, 0],[0, 0, 0, 0, 1, 0],[0, 0, 0, 0, 0, 1]])
        D=np.array([[0, 0],[0, 0],[0, 0],[0, 0]])

        # Discretise the system (forward Euler)
        Ad=np.identity(np.size(A,1))+Ts*A
        Bd=Ts*B
        Cd=C
        Dd=D

        return Ad, Bd, Cd, Dd

    def augmented_matrices(self, Ad, Bd, Cd, Dd):

        A_aug=np.concatenate((Ad,Bd),axis=1)
        temp1=np.zeros((np.size(Bd,1),np.size(Ad,1)))
        temp2=np.identity(np.size(Bd,1))
        temp=np.concatenate((temp1,temp2),axis=1)

        A_aug=np.concatenate((A_aug,temp),axis=0)
        B_aug=np.concatenate((Bd,np.identity(np.size(Bd,1))),axis=0)
        C_aug=np.concatenate((Cd,np.zeros((np.size(Cd,0),np.size(Bd,1)))),axis=1)
        D_aug=Dd

        return A_aug, B_aug, C_aug, D_aug

    def mpc_simplification(self, Ad, Bd, Cd, Dd, hz, x_aug_t, du):
        '''This function creates the compact matrices for Model Predictive Control'''
        # db - double bar
        # dbt - double bar transpose
        # dc - double circumflex

        A_aug, B_aug, C_aug, D_aug=self.augmented_matrices(Ad, Bd, Cd, Dd)

        Q=self.constants['Q']
        S=self.constants['S']
        R=self.constants['R']
        Cf=self.constants['Cf']
        g=self.constants['g']
        m=self.constants['m']
        mju=self.constants['mju']
        lf=self.constants['lf']
        inputs=self.constants['inputs']

        ############################### Constraints #############################
        d_delta_max=np.pi/300
        d_a_max=0.1
        d_delta_min=-np.pi/300
        d_a_min=-0.1

        ub_global=np.zeros(inputs*hz)
        lb_global=np.zeros(inputs*hz)

        # Only works for 2 inputs
        for i in range(0,inputs*hz):
            if i%2==0:
                ub_global[i]=d_delta_max
                lb_global[i]=-d_delta_min
            else:
                ub_global[i]=d_a_max
                lb_global[i]=-d_a_min

        ub_global=ub_global[0:inputs*hz]
        lb_global=lb_global[0:inputs*hz]
        ublb_global=np.concatenate((ub_global,lb_global),axis=0)

        I_global=np.eye(inputs*hz)
        I_global_negative=-I_global
        I_mega_global=np.concatenate((I_global,I_global_negative),axis=0)

        y_asterisk_max_global=[]
        y_asterisk_min_global=[]

        C_asterisk=np.matrix('1 0 0 0 0 0 0 0;\
                        0 1 0 0 0 0 0 0;\
                        0 0 0 0 0 0 1 0;\
                        0 0 0 0 0 0 0 1')

        C_asterisk_global=np.zeros((np.size(C_asterisk,0)*hz,np.size(C_asterisk,1)*hz))

        #########################################################################

        CQC=np.matmul(np.transpose(C_aug),Q)
        CQC=np.matmul(CQC,C_aug)

        CSC=np.matmul(np.transpose(C_aug),S)
        CSC=np.matmul(CSC,C_aug)

        QC=np.matmul(Q,C_aug)
        SC=np.matmul(S,C_aug)

        Qdb=np.zeros((np.size(CQC,0)*hz,np.size(CQC,1)*hz))
        Tdb=np.zeros((np.size(QC,0)*hz,np.size(QC,1)*hz))
        Rdb=np.zeros((np.size(R,0)*hz,np.size(R,1)*hz))
        Cdb=np.zeros((np.size(B_aug,0)*hz,np.size(B_aug,1)*hz))
        Adc=np.zeros((np.size(A_aug,0)*hz,np.size(A_aug,1)))

        ######################### Advanced LPV ##################################
        A_product=A_aug
        states_predicted_aug=x_aug_t
        A_aug_collection=np.zeros((hz,np.size(A_aug,0),np.size(A_aug,1)))
        B_aug_collection=np.zeros((hz,np.size(B_aug,0),np.size(B_aug,1)))
        #########################################################################

        for i in range(0,hz):
            if i == hz-1:
                Qdb[np.size(CSC,0)*i:np.size(CSC,0)*i+CSC.shape[0],np.size(CSC,1)*i:np.size(CSC,1)*i+CSC.shape[1]]=CSC
                Tdb[np.size(SC,0)*i:np.size(SC,0)*i+SC.shape[0],np.size(SC,1)*i:np.size(SC,1)*i+SC.shape[1]]=SC
            else:
                Qdb[np.size(CQC,0)*i:np.size(CQC,0)*i+CQC.shape[0],np.size(CQC,1)*i:np.size(CQC,1)*i+CQC.shape[1]]=CQC
                Tdb[np.size(QC,0)*i:np.size(QC,0)*i+QC.shape[0],np.size(QC,1)*i:np.size(QC,1)*i+QC.shape[1]]=QC

            Rdb[np.size(R,0)*i:np.size(R,0)*i+R.shape[0],np.size(R,1)*i:np.size(R,1)*i+R.shape[1]]=R

            ########################### Advanced LPV ############################
            Adc[np.size(A_aug,0)*i:np.size(A_aug,0)*i+A_aug.shape[0],0:0+A_aug.shape[1]]=A_product
            A_aug_collection[i][:][:]=A_aug
            B_aug_collection[i][:][:]=B_aug
            #####################################################################

            ######################## Constraints ################################
            x_dot_max=30
            if 0.17*states_predicted_aug[0][0] < 3:
                y_dot_max=0.17*states_predicted_aug[0][0]
            else:
                y_dot_max=3
            delta_max=np.pi/6
            Fyf=Cf*(states_predicted_aug[6][0]-states_predicted_aug[1][0]/states_predicted_aug[0][0]-lf*states_predicted_aug[3][0]/states_predicted_aug[0][0])
            a_max=1+(Fyf*np.sin(states_predicted_aug[6][0])+mju*m*g)/m-states_predicted_aug[3][0]*states_predicted_aug[1][0]
            x_dot_min=1
            if -0.17*states_predicted_aug[0][0] > -3:
                y_dot_min=-0.17*states_predicted_aug[0][0]
            else:
                y_dot_min=-3
            delta_min=-np.pi/6
            a_min=-4+(Fyf*np.sin(states_predicted_aug[6][0])+mju*m*g)/m-states_predicted_aug[3][0]*states_predicted_aug[1][0]

            y_asterisk_max=np.array([x_dot_max,y_dot_max,delta_max,a_max])
            y_asterisk_min=np.array([x_dot_min,y_dot_min,delta_min,a_min])

            y_asterisk_max_global=np.concatenate((y_asterisk_max_global,y_asterisk_max),axis=0)
            y_asterisk_min_global=np.concatenate((y_asterisk_min_global,y_asterisk_min),axis=0)

            C_asterisk_global[np.size(C_asterisk,0)*i:np.size(C_asterisk,0)*i+C_asterisk.shape[0],np.size(C_asterisk,1)*i:np.size(C_asterisk,1)*i+C_asterisk.shape[1]]=C_asterisk


            #####################################################################

            ######################### Advanced LPV ##############################
            if i<hz-1:
                du1=du[inputs*(i+1)][0]
                du2=du[inputs*(i+1)+inputs-1][0]
                states_predicted_aug=np.matmul(A_aug,states_predicted_aug)+np.matmul(B_aug,np.transpose([[du1,du2]]))
                states_predicted=np.transpose(states_predicted_aug[0:6])[0]
                delta_predicted=states_predicted_aug[6][0]
                a_predicted=states_predicted_aug[7][0]
                Ad, Bd, Cd, Dd=self.state_space(states_predicted,delta_predicted,a_predicted)
                A_aug, B_aug, C_aug, D_aug=self.augmented_matrices(Ad, Bd, Cd, Dd)
                A_product=np.matmul(A_aug,A_product)

        for i in range(0,hz):
            for j in range(0,hz):
                if j<=i:
                    AB_product=np.eye(np.shape(A_aug)[0])
                    for ii in range(i,j-1,-1):
                        if ii>j:
                            AB_product=np.matmul(AB_product,A_aug_collection[ii][:][:])
                        else:
                            AB_product=np.matmul(AB_product,B_aug_collection[ii][:][:])
                    Cdb[np.size(B_aug,0)*i:np.size(B_aug,0)*i+B_aug.shape[0],np.size(B_aug,1)*j:np.size(B_aug,1)*j+B_aug.shape[1]]=AB_product

        #########################################################################

        ####################### Constraints #####################################

        Cdb_constraints=np.matmul(C_asterisk_global,Cdb)
        Cdb_constraints_negative=-Cdb_constraints
        Cdb_constraints_global=np.concatenate((Cdb_constraints,Cdb_constraints_negative),axis=0)

        Adc_constraints=np.matmul(C_asterisk_global,Adc)
        Adc_constraints_x0=np.transpose(np.matmul(Adc_constraints,x_aug_t))[0]
        y_max_Adc_difference=y_asterisk_max_global-Adc_constraints_x0
        y_min_Adc_difference=-y_asterisk_min_global+Adc_constraints_x0
        y_Adc_difference_global=np.concatenate((y_max_Adc_difference,y_min_Adc_difference),axis=0)

        G=np.concatenate((I_mega_global,Cdb_constraints_global),axis=0)
        ht=np.concatenate((ublb_global,y_Adc_difference_global),axis=0)

        #######################################################################

        Hdb=np.matmul(np.transpose(Cdb),Qdb)
        Hdb=np.matmul(Hdb,Cdb)+Rdb

        temp=np.matmul(np.transpose(Adc),Qdb)
        temp=np.matmul(temp,Cdb)

        temp2=np.matmul(-Tdb,Cdb)
        Fdbt=np.concatenate((temp,temp2),axis=0)

        return Hdb,Fdbt,Cdb,Adc,G,ht

    def open_loop_new_states(self,states,delta,a):
        '''This function computes the new state vector for one sample time later'''

        # Get the necessary constants
        g=self.constants['g']
        m=self.constants['m']
        Iz=self.constants['Iz']
        Cf=self.constants['Cf']
        Cr=self.constants['Cr']
        lf=self.constants['lf']
        lr=self.constants['lr']
        Ts=self.constants['Ts']
        mju=self.constants['mju']

        current_states=states
        new_states=current_states
        x_dot=current_states[0]
        y_dot=current_states[1]
        psi=current_states[2]
        psi_dot=current_states[3]
        X=current_states[4]
        Y=current_states[5]

        sub_loop=30  #Chops Ts into 30 pieces
        for i in range(0,sub_loop):

            # Compute lateral forces
            Fyf=Cf*(delta-y_dot/x_dot-lf*psi_dot/x_dot)
            Fyr=Cr*(-y_dot/x_dot+lr*psi_dot/x_dot)

            # Compute the the derivatives of the states
            x_dot_dot=a+(-Fyf*np.sin(delta)-mju*m*g)/m+psi_dot*y_dot
            y_dot_dot=(Fyf*np.cos(delta)+Fyr)/m-psi_dot*x_dot
            psi_dot=psi_dot
            psi_dot_dot=(Fyf*lf*np.cos(delta)-Fyr*lr)/Iz
            X_dot=x_dot*np.cos(psi)-y_dot*np.sin(psi)
            Y_dot=x_dot*np.sin(psi)+y_dot*np.cos(psi)

            # Update the state values with new state derivatives
            x_dot=x_dot+x_dot_dot*Ts/sub_loop
            y_dot=y_dot+y_dot_dot*Ts/sub_loop
            psi=psi+psi_dot*Ts/sub_loop
            psi_dot=psi_dot+psi_dot_dot*Ts/sub_loop
            X=X+X_dot*Ts/sub_loop
            Y=Y+Y_dot*Ts/sub_loop

        # Take the last states
        new_states[0]=x_dot
        new_states[1]=y_dot
        new_states[2]=psi
        new_states[3]=psi_dot
        new_states[4]=X
        new_states[5]=Y

        return new_states,x_dot_dot,y_dot_dot,psi_dot_dot
