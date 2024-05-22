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
        Ts=0.1
        mju=0.02 # friction coefficient

        ####################### Lateral control #################################

        outputs=4 # number of outputs
        inputs=2 # number of inputs
        hz = 10 # horizon period

        trajectory=4 # Choose 1, 2 or 3, nothing else
        version=2 # This is only for trajectory 3 (Choose 1 or 2)

        # Matrix weights for the cost function (They must be diagonal)

        if trajectory==3 and version==2:  #3rd trajectoris with larger distances
            # Weights for trajectory 3, version 2
            Q=np.matrix('100 0 0 0;0 20000 0 0;0 0 1000 0;0 0 0 1000') # weights for outputs (all samples, except the last one)
            S=np.matrix('100 0 0 0;0 20000 0 0;0 0 1000 0;0 0 0 1000') # weights for the final horizon period outputs
            R=np.matrix('100 0;0 1') # weights for inputs
        elif trajectory==3 or trajectory==4 or trajectory==5:
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
            first_section=14  # 14 sec for first part of the trajectory
            other_sections=14  # 14 sec for other parts of the trajectories
            time_length=first_section+other_sections*10  # total time length of 11 trajectories
            delay=np.zeros(12)  

            # this loop defines the time start and end of each trajectories
            for dly in range(1,len(delay)):
                delay[dly]=first_section+(dly-1)*other_sections

            #uncomment below to see the time spans
            # print(delay)
            # exit()  
        elif trajectory == 4:
            time_length = 90  #54
            x_lim = 170
            y_lim = 160

        elif trajectory == 5:
            time_length = 26
            x_lim = 170
            y_lim = 160

        else:
            print("trajectory: 1,2 or 3; version: 1 or 2")

        self.constants={'g':g,'m':m,'Iz':Iz,'Cf':Cf,'Cr':Cr,'lf':lf,'lr':lr,\
        'Ts':Ts,'mju':mju,'Q':Q,'S':S,'R':R,'outputs':outputs,'inputs':inputs,\
        'hz':hz,'delay':delay,'time_length':time_length,'trajectory':trajectory,\
        'version':version,'x_lim':x_lim,'y_lim':y_lim}
        # exit()
        return None
    
    def interpolate_waypoints(self, waypoints, num_intermediate_points=3):  ### 13 for trajector=4 and 9 for traj 5
        interpolated_waypoints = []
        for i in range(len(waypoints) - 1):
            lat1, lon1 = waypoints[i]
            lat2, lon2 = waypoints[i + 1]
            xs = np.linspace(lat1, lat2, num=num_intermediate_points + 2)[1:-1]
            ys = np.linspace(lon1, lon2, num=num_intermediate_points + 2)[1:-1]
            interpolated_waypoints.extend(list(zip(xs, ys)))

        return interpolated_waypoints



    def generate_trajectory(self,waypoints):
        # Unpack latitude and longitude from waypoints
        lat, lon = zip(*waypoints)
        cs_lat = CubicSpline(range(len(lat)), lat)  # Cubic spline interpolation for latitude
        cs_lon = CubicSpline(range(len(lon)), lon)  # Cubic spline interpolation for longitude

        # Generate a denser trajectory for smoother transitions
        spline_points = np.linspace(0, len(lat) - 1, num=len(lat)*1) #600 ,30
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

    def moving_average(self, data, window_size):
        return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

    def exponential_moving_average(self, data, alpha):
        ema = np.zeros_like(data)
        ema[0] = data[0]
        for i in range(1, len(data)):
            ema[i] = alpha * data[i] + (1 - alpha) * ema[i - 1]
        return ema
    
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
            from scipy.ndimage import gaussian_filter1d
            waypoints = [[195035.01564388964,1948392.360874124],
                        [195035.00679082592,1948392.6809574235],
                        [195034.9936108887,1948393.1645646323],
                        [195034.98011441834,1948393.7005877814],
                        [195034.9654643212,1948394.2330837764],
                        [195034.94940859164,1948394.8093175283],
                        [195034.9349560018,1948395.3659077387],
                        [195034.91456574108,1948396.0940011062],
                        [195034.89572758143,1948396.8022700367],
                        [195034.8738637947,1948397.63090961],
                        [195034.8562683566,1948398.2652426867],
                        [195034.83166330607,1948399.1816635379],
                        [195034.80626529056,1948400.1469317852],
                        [195034.7875723583,1948400.9717105723],
                        [195034.76831182337,1948401.922887735],
                        [195034.7486332703,1948403.0092317807],
                        [195034.73361898478,1948404.0334963226],
                        [195034.7192354125,1948404.9831489022],
                        [195034.70736153112,1948406.0721953758],
                        [195034.69722293824,1948407.3044088199],
                        [195034.68968415333,1948408.2142206365],
                        [195034.6828284861,1948409.479757933],
                        [195034.6774374092,1948410.64275134],
                        [195034.6732401281,1948411.9286749694],
                        [195034.6698572588,1948412.9816180533],
                        [195034.66653711168,1948414.151951868],
                        [195034.66353066007,1948415.4367628216],
                        [195034.66332326047,1948416.7168819457],
                        [195034.66579409837,1948417.6439161],
                        [195034.6689763078,1948418.797394651],
                        [195034.67394859166,1948420.0605990828],
                        [195034.67921436354,1948421.3177413528],
                        [195034.68235747935,1948422.228656699],
                        [195034.68453294094,1948423.4779424388],
                        [195034.68779055562,1948424.719872987],
                        [195034.69023382966,1948425.6195602077],
                        [195034.69213884868,1948426.8557623469],
                        [195034.6952681908,1948428.0910869876],
                        [195034.69602101532,1948429.212847185],
                        [195034.69640940512,1948430.111945304],
                        [195034.6977127925,1948431.3501082882],
                        [195034.69809805622,1948432.5897393809],
                        [195034.69968302792,1948433.4917971103],
                        [195034.70107520858,1948434.7316391405],
                        [195034.70110240247,1948435.971377978],
                        [195034.69932186234,1948437.0994017066],
                        [195034.6986627423,1948438.1159837688],
                        [195034.696465182,1948439.3593449427],
                        [195034.69677004858,1948440.2645279274],
                        [195034.69602271463,1948441.5094175055],
                        [195034.6983281464,1948442.75633115],
                        [195034.70000905736,1948443.7782220934],
                        [195034.70191149,1948445.0283038751],
                        [195034.7040841457,1948446.1673257127],
                        [195034.70304908507,1948447.1931936746],
                        [195034.700150772,1948448.3357433467],
                        [195034.7009689791,1948449.5983115435],
                        [195034.70242482048,1948450.8635813135],
                        [195034.7037826161,1948451.787410464],
                        [195034.70040720285,1948453.063102208],
                        [195034.69902683859,1948454.2248547145],
                        [195034.69890064135,1948455.5097635416],
                        [195034.69765159627,1948456.4474671546],
                        [195034.69742816733,1948457.740219313],
                        [195034.7014150095,1948459.0400235036],
                        [195034.70623183527,1948459.989290092],
                        [195034.7129349051,1948461.2976449335],
                        [195034.719238495,1948462.4898587125],
                        [195034.7250103911,1948463.8050410568],
                        [195034.72792718516,1948464.8825897623],
                        [195034.73031946796,1948466.0809376403],
                        [195034.7339184442,1948467.4025806612],
                        [195034.7371984694,1948468.6088445152],
                        [195034.73955317534,1948469.6988103215],
                        [195034.74159657056,1948470.9133375515],
                        [195034.74429056316,1948472.2525505123],
                        [195034.7470667227,1948473.595431753],
                        [195034.7499285604,1948474.6975523017],
                        [195034.7542011797,1948475.9264459964],
                        [195034.76329808898,1948477.1580439033],
                        [195034.7757598747,1948478.5145430865],
                        [195034.782435825,1948479.5050003703],
                        [195034.79429500073,1948480.995756896],
                        [195034.80231105385,1948482.1146876037],
                        [195034.80891656972,1948483.487892778],
                        [195034.81369728217,1948484.6148181641],
                        [195034.8188895589,1948485.8691229548],
                        [195034.82449420023,1948487.2521319757],
                        [195034.82594753808,1948488.38629157],
                        [195034.82520674675,1948489.6447881223],
                        [195034.82378931352,1948491.0278841988],
                        [195034.82218880206,1948492.0321570209],
                        [195034.82037636286,1948493.4106896562],
                        [195034.81910405296,1948494.7866052368],
                        [195034.8178927635,1948495.7843505298],
                        [195034.81858386652,1948497.1497809975],
                        [195034.8188284955,1948498.5085088608],
                        [195034.81783023616,1948499.8623689995],
                        [195034.81691997117,1948500.8431316374],
                        [195034.81065115926,1948502.1846100804],
                        [195034.80133570152,1948503.5182354606],
                        [195034.7936096971,1948504.4843682607],
                        [195034.78232972434,1948505.92543568],
                        [195034.7742318963,1948506.9928723178],
                        [195034.76375203786,1948508.2757697494],
                        [195034.75676322557,1948509.189727439],
                        [195034.7471574447,1948510.4169514647],
                        [195034.73861274484,1948511.6097658707],
                        [195034.73088937884,1948512.6639397293],
                        [195034.72528226284,1948513.5943910496],
                        [195034.72021825958,1948514.6149987075],
                        [195034.71371760246,1948515.7280926306],
                        [195034.71058261138,1948516.731521801],
                        [195034.709222238,1948517.629698746],
                        [195034.71206668075,1948518.7178366412],
                        [195034.72059404213,1948519.701958078],
                        [195034.7326701871,1948520.5831165626],
                        [195034.75632809487,1948521.6552856162],
                        [195034.78579610938,1948522.5277072478],
                        [195034.831159493,1948523.4926014952],
                        [195034.9038048816,1948524.5461857342],
                        [195034.99331500664,1948525.49629722],
                        [195035.09476675538,1948526.3437573041],
                        [195035.23622446417,1948527.2759265928],
                        [195035.42788619426,1948528.2816059478],
                        [195035.63326609068,1948529.1833817349],
                        [195035.89384396985,1948530.162083081],
                        [195036.13426753122,1948530.953685912],
                        [195036.43231645218,1948531.8192776812],
                        [195036.7943819962,1948532.755087252],
                        [195037.0818450105,1948533.4241099039],
                        [195037.51419646866,1948534.326540901],
                        [195037.98715491404,1948535.2070747048],
                        [195038.3603707729,1948535.8326618627],
                        [195038.90661293553,1948536.670737108],
                        [195039.49551406395,1948537.479579046],
                        [195040.06152899994,1948538.1932589116],
                        [195040.60018341863,1948538.814682621],
                        [195041.29552039903,1948539.5491855815],
                        [195041.95953390404,1948540.19070655],
                        [195042.5827267875,1948540.7448442536],
                        [195043.36996616912,1948541.396574966],
                        [195044.10719225946,1948541.9683006],
                        [195044.78686385808,1948542.4652083819],
                        [195045.71338436846,1948543.1140614438],
                        [195046.42292564065,1948543.5868314877],
                        [195047.14440841807,1948544.046195385],
                        [195048.04214365338,1948544.5891163442],
                        [195048.87180083472,1948545.0632551708],
                        [195049.8013801048,1948545.5656016567],
                        [195050.4878600191,1948545.915071205],
                        [195051.44241937326,1948546.3774785185],
                        [195052.41138089285,1948546.812987828],
                        [195053.12178032874,1948547.1154396008],
                        [195054.10582817515,1948547.5113618488],
                        [195055.09839214775,1948547.879964965],
                        [195055.82475799887,1948548.132915209],
                        [195056.83017796642,1948548.4548306502],
                        [195057.8418555005,1948548.745708013],
                        [195058.76627521135,1948548.9813534447],
                        [195059.6952304908,1948549.188363343],
                        [195060.53488996762,1948549.3483804807],
                        [195061.56481812065,1948549.5109545467],
                        [195062.31594803635,1948549.6055484624],
                        [195063.3480989634,1948549.7006854212],
                        [195064.47515723036,1948549.7610082915],
                        [195065.41586799215,1948549.7781149019],
                        [195066.1687667167,1948549.7696849979],
                        [195067.20282615448,1948549.725724591],
                        [195068.2335200854,1948549.6464895864],
                        [195069.1695739597,1948549.5440109414],
                        [195070.00785532186,1948549.424268417],
                        [195071.0282082099,1948549.2455143258],
                        [195071.85819255988,1948549.0727575247],
                        [195072.77591783885,1948548.8552955631],
                        [195073.77582068933,1948548.5815729478],
                        [195074.49749138975,1948548.360349162],
                        [195075.48114335493,1948548.0254916481],
                        [195076.4520635102,1948547.6552299045],
                        [195077.14964429324,1948547.3622260615],
                        [195078.0968668081,1948546.9287565046],
                        [195079.03209904628,1948546.4600666398],
                        [195079.78960278936,1948546.0493220729],
                        [195080.6200444467,1948545.5663377745],
                        [195081.5196282756,1948545.0042151213],
                        [195082.1624728788,1948544.5738148035],
                        [195083.03014879854,1948543.9527418811],
                        [195083.876748217,1948543.2977672566],
                        [195084.62915633907,1948542.6730107134],
                        [195085.43705463377,1948541.9524604054],
                        [195086.00902661684,1948541.4083692695],
                        [195086.77443435957,1948540.6370889605],
                        [195087.50767514756,1948539.8401046298],
                        [195088.01907892205,1948539.239541788],
                        [195088.6952809101,1948538.3860315867],
                        [195089.33107410895,1948537.5046511681],
                        [195089.76626299878,1948536.85017003],
                        [195090.37551000077,1948535.8440295116],
                        [195090.79321290305,1948535.068366762],
                        [195091.252916189,1948534.0945944632],
                        [195091.58857428312,1948533.2796045702],
                        [195091.91981576462,1948532.359394934],
                        [195092.20509433432,1948531.4242263492],
                        [195092.4661992804,1948530.3816794048],
                        [195092.65081266375,1948529.424486945],
                        [195092.77848941024,1948528.5573603276],
                        [195092.88258416986,1948527.4927375477],
                        [195092.9290675209,1948526.6204869791],
                        [195092.9405877809,1948525.6486721637],
                        [195092.91649399968,1948524.5758477214],
                        [195092.86780299214,1948523.5990876164],
                        [195092.8089270887,1948522.7188552686],
                        [195092.71788929493,1948521.5421940219],
                        [195092.6456962597,1948520.6560220867],
                        [195092.55675080078,1948519.5674908157],
                        [195092.49107658054,1948518.7743067257],
                        [195092.40043014512,1948517.6816605385],
                        [195092.3045944532,1948516.5861065185],
                        [195092.23088009853,1948515.7848315723],
                        [195092.12332287373,1948514.6769890317],
                        [195092.01135172482,1948513.5650797163],
                        [195091.92919682842,1948512.7551248332],
                        [195091.81296879952,1948511.6387593083],
                        [195091.6965143742,1948510.5191231077],
                        [195091.58786496468,1948509.5017759772],
                        [195091.4808197793,1948508.4813268087],
                        [195091.38401759096,1948507.5586915438],
                        [195091.26925799815,1948506.4260591338],
                        [195091.17312914616,1948505.4945761913],
                        [195091.06772754618,1948504.4514408617],
                        [195090.9533332107,1948503.298932881],
                        [195090.8400057936,1948502.1383785522],
                        [195090.75799679858,1948501.2899319855],
                        [195090.63618434244,1948500.0128387958],
                        [195090.5480458351,1948499.0513369206],
                        [195090.45959599546,1948498.0859020944],
                        [195090.35290563348,1948496.9006476167],
                        [195090.25586987703,1948495.817086649],
                        [195090.14908941468,1948494.6192099287],
                        [195090.0730417943,1948493.7438234377],
                        [195089.96672105085,1948492.535023236],
                        [195089.85951079216,1948491.3179948598],
                        [195089.7803771094,1948490.427600586],
                        [195089.66531120086,1948489.199914447],
                        [195089.54679609864,1948487.9714801093],
                        [195089.45799875882,1948487.0819499823],
                        [195089.33804091337,1948485.8746065933],
                        [195089.229791801,1948484.7966286154],
                        [195089.11362108146,1948483.6380631207],
                        [195089.0206140953,1948482.7144726666],
                        [195088.9213464876,1948481.7171023635],
                        [195088.81332868952,1948480.659503327],
                        [195088.72793024188,1948479.8286236404],
                        [195088.63401600078,1948478.9454959764],
                        [195088.5305809025,1948478.0231354942],
                        [195088.43095255716,1948477.2322444322],
                        [195088.3386622715,1948476.5563085398],
                        [195088.2121772462,1948475.774538129],
                        [195088.07644522964,1948475.103320459],
                        [195087.90622410004,1948474.4033679669],
                        [195087.75951982517,1948473.918305262],
                        [195087.5402546288,1948473.2845880105],
                        [195087.34079810482,1948472.797539055],
                        [195087.09792971806,1948472.292014369],
                        [195086.80586929782,1948471.7715466851],
                        [195086.50912620136,1948471.3216174685],
                        [195086.14538460178,1948470.8463583516],
                        [195085.8205999418,1948470.4727917204],
                        [195085.4287319146,1948470.0744980038],
                        [195084.9634273453,1948469.6665307132],
                        [195084.60317679372,1948469.3875726757],
                        [195084.0824559178,1948469.0251286675],
                        [195083.53093749352,1948468.6983011232],
                        [195083.11677315205,1948468.4796479144],
                        [195082.5264640307,1948468.2023898482],
                        [195081.91519507454,1948467.9489729297],
                        [195081.3439639897,1948467.7443047394],
                        [195080.8174331957,1948467.5750654205],
                        [195080.1586360029,1948467.3749014179],
                        [195079.54517944573,1948467.205952328],
                        [195078.85899341406,1948467.0274481378],
                        [195078.35300699424,1948466.8978774573],
                        [195077.64586663054,1948466.726100755],
                        [195076.91886897158,1948466.5665358263],
                        [195076.37939635816,1948466.4599984838],
                        [195075.5590197736,1948466.313799604],
                        [195074.93149595533,1948466.2202380328],
                        [195074.15538206205,1948466.123946265],
                        [195073.51411108222,1948466.0577980352],
                        [195072.79608898988,1948465.9982054327],
                        [195072.00410144348,1948465.947881726],
                        [195071.42926508127,1948465.9176276436],
                        [195070.65263596002,1948465.881470933],
                        [195069.89502594247,1948465.8489781304],
                        [195069.22446380596,1948465.821799453],
                        [195068.57471504758,1948465.790435079],
                        [195068.0089326343,1948465.7614978834],
                        [195067.34192982578,1948465.7289445212],
                        [195066.87308584564,1948465.7043837062],
                        [195066.2514221017,1948465.6703109122],
                        [195065.61004285805,1948465.6344626064],
                        [195065.15351874684,1948465.609895166],
                        [195064.6251665201,1948465.579244393],
                        [195064.26524504754,1948465.55784049],
                        [195063.81270972406,1948465.532095324],
                        [195063.48862575812,1948465.5136470622],
                        [195063.18502820263,1948465.4960346678],
                        [195062.9233667608,1948465.4815063542],
                        [195062.74243414073,1948465.4735664611],
                        [195062.66349119085,1948465.4700626782]]
            # print(len(waypoints))
            for ss in waypoints:
                ss.reverse()
            #####################################
            waypoints = self.interpolate_waypoints(waypoints)
            # X,Y,yaw,ck = self.generate_trajectory(waypoints)
            ####################################
            # f_x=np.array([4,80,135,159,157,142,124,91,75,75])*2 # x end ponts for sub trajectories
            # f_y=np.array([4,4,5,27,43,61,60,57,50,40])*2 # y end ponts for sub trajectories

            # points_list = [[x, y] for x, y in zip(f_x, f_y)]
            
            # X, Y = self.gps_to_xy(waypoints_updated)
            # print(len(X),len(Y))
            # points_list = [[x, y] for x, y in zip(X, Y)]
            X=[]
            Y=[]
            for wp in waypoints:
                X.append(wp[0])
                Y.append(wp[1])
            # print(len(xtest),len(ytest))
            # X,Y,yaw,ck = self.generate_trajectory(points_list)
            print("X,Y length: ",len(X),len(Y))
            # X = np.append(X,X[-1])
            # Y = np.append(Y,Y[-1])
            
            print("time length: ",len(t))
            print("final X,Y length: ",len(X),len(Y))
            # window_size = 20 # Adjust this parameter as needed
            # dX_smooth = self.moving_average(X, window_size)
            # dY_smooth = self.moving_average(Y, window_size)
            sigma = 10  # Standard deviation for Gaussian kernel
            X = gaussian_filter1d(X, sigma)
            Y = gaussian_filter1d(Y, sigma)
            # alpha = 0.02  # Smoothing factor
            # X = self.exponential_moving_average(X, alpha)
            # Y = self.exponential_moving_average(Y, alpha)
            X = np.array(X[26:])
            Y = np.array(Y[26:])
            print("asdsdfa: ",len(X))

            # exit()
            ################
            # Plot the world
            plt.plot(X,Y,'b',linewidth=2,label='The trajectory')
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
            plt.show()
            # exit()

        elif trajectory ==5:
            waypoints = [[1.5063,-0.0824],
[3.0199,-0.11],
[4.6484,-0.076],
[6.3053,-0.0942],
[7.9286,-0.0805],
[9.5731,-0.0852],
[11.11,-0.0816],
[12.7052,-0.0949],
[14.2665,-0.061],
[15.785,-0.0852],
[17.4477,-0.0381],
[19.0761,0.1005],
[20.6613,0.3906],
[22.1659,1.0582],
[23.4606,1.986],
[24.4448,3.1196],
[25.1466,4.6295],
[25.4468,6.1326],
[25.6464,7.6315],
[25.7876,9.1511],
[25.9527,10.6991],
[26.1443,12.3785],
[26.3251,13.9092],
[26.499,15.4878],
[26.6771,17.0808],
[26.8153,18.6713],
[27.0318,20.2666],
[27.2754,21.8552],
[27.518,23.3844],
[27.7646,24.9148],
[28.0088,26.518],
[28.2122,28.0863],
[28.4379,29.7225],
[28.642,31.3095],
[28.8324,32.8755],
[29.0069,34.4595],
[29.1858,36.0261],
[29.4115,37.6058],
[29.6679,39.1918],
[29.924,40.7569],
[30.1581,42.3427],
[30.4004,43.9163],
[30.6019,45.5131],
[30.766,47.0952],
[30.9368,48.7525],
[31.106,50.4104],
[31.2485,52.0895],
[31.5378,53.6229],
[32.1857,55.2084],
[31.8479,56.7319],
[32.172,58.2351],
[32.448,59.8663],
[32.6642,61.4924],
[32.7622,63.1594],
[32.6337,64.6853],
[32.2748,66.2668],
[31.5891,67.7712],
[30.7539,69.224],
[29.7346,70.5273],
[28.6686,71.7589],
[27.4088,72.966],
[26.1357,74.0783],
[24.8386,75.1214],
[23.538,76.1779],
[22.2236,77.2052],
[20.861,78.1221],
[19.487,78.9453],
[18.0758,79.683],
[16.6168,80.3256],
[15.1233,80.858],
[13.5873,81.319],
[12.076,81.6729],
[10.5401,81.9747],
[8.938,82.1981],
[7.1806,82.3498],
[5.5843,82.426],
[3.9463,82.3051],
[2.3487,82.1306],
[0.7806,81.8922],
[-0.7925,81.5633],
[-2.3264,81.2146],
[-3.8783,80.7569],
[-5.4235,80.2513],
[-6.9289,79.6537],
[-8.5224,78.924],
[-9.9845,78.2289],
[-11.3669,77.4535],
[-12.7288,76.6567],
[-14.0568,75.8047],
[-15.326,74.9231],
[-16.5648,74.0214],
[-17.7655,73.1099],
[-18.9309,72.0935],
[-20.2669,71.014],
[-21.3798,69.8484],
[-22.3362,68.447],
[-23.002,66.9881],
[-23.4674,65.4413],
[-23.7457,63.891],
[-23.9315,62.3174],
[-24.0811,60.7498],
[-24.1626,59.0993],
[-24.211,57.5708],
[-24.1988,55.9003],
[-24.1898,54.2977],
[-24.1641,52.7972],
[-24.119,51.2878],
[-24.096,49.7503],
[-24.1233,48.0574],
[-24.158,46.4608],
[-24.231,44.816],
[-24.3206,43.2073],
[-24.3948,41.5394],
[-24.5105,39.8821],
[-24.6423,38.1581],
[-24.7499,36.6548],
[-24.8514,35.128],
[-24.9508,33.6148],
[-25.0458,32.0471],
[-25.1248,30.3407],
[-25.1754,28.8017],
[-25.2337,27.2711],
[-25.2982,25.7361],
[-25.3277,24.2098],
[-25.3308,22.7032],
[-25.3534,20.9664],
[-25.3575,19.2328],
[-25.4014,17.4568],
[-25.4259,15.9358],
[-25.4599,14.3596],
[-25.4864,12.8552],
[-25.5232,11.1961],
[-25.5255,9.5057],
[-25.3942,7.9021],
[-25.0681,6.3788],
[-24.5513,4.9248],
[-23.681,3.5102],
[-22.517,2.3031],
[-21.2591,1.4694],
[-19.6502,0.9217],
[-18.1087,0.6355],
[-16.5562,0.5582],
[-14.9323,0.5433],
[-13.2879,0.5405],
[-11.6768,0.4895],
[-10.1015,0.4782],
[-8.4711,0.3717],
[-6.9734,0.2829]]

            for ss in waypoints:
                ss.reverse()

            print("length of waypoints: ",len(waypoints))
            # points_array = np.array(waypoints)
            
            # # Separate X and Y coordinates
            # x = points_array[:, 0]
            # y = points_array[:, 1]

            # # Calculate the decimation factor
            # decimation_factor = len(x) // 500  # Determine the decimation factor to reduce 2909 points to 1000

            # # Decimate points
            # decimated_x = x[::decimation_factor]
            # decimated_y = y[::decimation_factor]
            # print("length after decimation: ",len(decimated_x))
            # print(len(waypoints))
            # f_x=np.array([0,80,110,140,160,110,40,10,40,70,110,150]) *2 # x end ponts for sub trajectories
            # f_y=np.array([40,20,20,60,100,140,140,80,60,60,90,90])*2
            # points_list = [[x, y] for x, y in zip(decimated_x, decimated_y)]
            
            #####################################
            waypoints_updated = self.interpolate_waypoints(waypoints)
            ####################################
            # f_x=np.array([4,80,135,159,157,142,124,91,75,75])*2 # x end ponts for sub trajectories
            # f_y=np.array([4,4,5,27,43,61,60,57,50,40])*2 # y end ponts for sub trajectories

            # points_list = [[x, y] for x, y in zip(f_x, f_y)]
            
            # X, Y = self.gps_to_xy(waypoints_updated)
            # print(len(X),len(Y))
            # points_list = [[x, y] for x, y in zip(X, Y)]
            X=[]
            Y=[]
            for wp in waypoints_updated:
                X.append(wp[0])
                Y.append(wp[1])
            # print(len(xtest),len(ytest))
            # X,Y,yaw,ck = self.generate_trajectory(waypoints)
            print("required_length: ",len(t))
            print("length after interpolation: ",len(X),len(Y))
            # X = np.append(X,X[-1])
            # Y = np.append(Y,Y[-1])
            X = np.array(X[22:])
            Y = np.array(Y[22:])
            # print(type(np.array(X)))
            
            print(len(X),len(Y))
            # print(len(X))
            # print(X.shape)
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
            f_x=np.array([0,60,110,140,160,110,40,10,40,70,110,150])*version # x end ponts for sub trajectories
            f_y=np.array([40,20,20,60,100,140,140,80,60,60,90,90])*version # y end ponts for sub trajectories

            # X & Y derivatives
            # f_x_dot is not X_dot this is just to calculate the trajectory
            f_x_dot=np.array([2,1,1,1,0,-1,-1,0,1,1,1,1])*3*version # x_dot end points, 3 is just to increase the magntude of velocity
            f_y_dot=np.array([0,0,0,1,1,0,0,-1,0,0,0,0])*3*version

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
            # exit()
            
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
        # np.save('X_p1.npy', X)
        # np.save('Y_p1.npy', Y)
        # calculate X_dot and Y_dot, Ts = 0.02
        X_dot=dX/Ts
        Y_dot=dY/Ts

        # length of X_dot and Y_dot is one less than X and Y, hence to make it of same length we concatinate first value twice
        X_dot=np.concatenate(([X_dot[0]],X_dot),axis=0)
        Y_dot=np.concatenate(([Y_dot[0]],Y_dot),axis=0)

        # print("xdot: ",len(X_dot), len(X))
        # plt.plot(t,X_dot,'b',linewidth=2,label='X_dot ref')
        # plt.plot(t,Y_dot,'r',linewidth=2,label='Y_dot ref')
        # plt.show()
        # print(X_dot[1000])
        # for i in range(len(X_dot)):
            # print("X_dot[{}:{}]: {}".format(i, X_dot[i]))
            # print(X_dot[1000])
            # print(Y_dot[1000])
            # print("Y_dot[{}:{}]: {}".format(i, Y_dot[i]))
        # print("X_dot: ",X_dot)
        # print("Y_dot: ",Y_dot)
        # Define the reference yaw angles
        psi=np.zeros(len(X))
        psiInt=psi
        psi[0]=np.arctan2(dY[0],dX[0])  # use arctan2 because it takes care of quadrant values
        psi[1:len(psi)]=np.arctan2(dY[0:len(dY)],dX[0:len(dX)])
        # print("psi: ",psi)
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
        # print("psiINT: ",psiInt)
        # calculate X_dot and Y_dot in body frame from inertial frame using rotation matrix as discused
        x_dot_body=np.cos(psiInt)*X_dot+np.sin(psiInt)*Y_dot
        x_dot_body[0] = 1.0
        y_dot_body=-np.sin(psiInt)*X_dot+np.cos(psiInt)*Y_dot
        y_dot_body=np.round(y_dot_body)  # round to avoid small numerical error

        # # Plot the body frame velocity
        # # plt.plot(t,x_dot_body,'g',linewidth=2,label='x_dot ref')
        plt.plot(t,X_dot,'b',linewidth=2,label='X_dot ref')
        plt.plot(t,Y_dot,'r',linewidth=2,label='Y_dot ref')
        # plt.xlabel('t [s]',fontsize=15)
        # plt.ylabel('X_dot_ref, Y_dot_ref [m/s]',fontsize=15)
        # plt.grid(True)
        # plt.legend(loc='upper right',fontsize='small')
        plt.show()
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

        sub_loop=10  #Chops Ts into 30 pieces
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
        new_states[4]=X  # pass gps x and y coordinate only
        new_states[5]=Y
        # print("new_states: ",new_states)
        # print("psi_value: ",np.rad2deg(psi))
        return new_states,x_dot_dot,y_dot_dot,psi_dot_dot
