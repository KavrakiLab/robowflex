/* Author: Zachary Kingston, Constantinos Chamzas */

#include <cmath>
#include <robowflex_library/constants.h>
#include <robowflex_library/io/colormap.h>
#include <robowflex_library/tf.h>

// From http://www.kennethmoreland.com/color-advice/ and
// https://ai.googleblog.com/2019/08/turbo-improved-rainbow-colormap-for.html

namespace
{
    struct Entry
    {
        double s;
        double r;
        double g;
        double b;
    };

    static const Entry TURBO[256] = {
        {0.00000000, 0.18995, 0.07176, 0.23217}, {0.00781250, 0.19483, 0.08339, 0.26149},
        {0.01171875, 0.19956, 0.09498, 0.29024}, {0.01562500, 0.20415, 0.10652, 0.31844},
        {0.01953125, 0.20860, 0.11802, 0.34607}, {0.02343750, 0.21291, 0.12947, 0.37314},
        {0.02734375, 0.21708, 0.14087, 0.39964}, {0.03125000, 0.22111, 0.15223, 0.42558},
        {0.03515625, 0.22500, 0.16354, 0.45096}, {0.03906250, 0.22875, 0.17481, 0.47578},
        {0.04296875, 0.23236, 0.18603, 0.50004}, {0.04687500, 0.23582, 0.19720, 0.52373},
        {0.05078125, 0.23915, 0.20833, 0.54686}, {0.05468750, 0.24234, 0.21941, 0.56942},
        {0.05859375, 0.24539, 0.23044, 0.59142}, {0.06250000, 0.24830, 0.24143, 0.61286},
        {0.06640625, 0.25107, 0.25237, 0.63374}, {0.07031250, 0.25369, 0.26327, 0.65406},
        {0.07421875, 0.25618, 0.27412, 0.67381}, {0.07812500, 0.25853, 0.28492, 0.69300},
        {0.08203125, 0.26074, 0.29568, 0.71162}, {0.08593750, 0.26280, 0.30639, 0.72968},
        {0.08984375, 0.26473, 0.31706, 0.74718}, {0.09375000, 0.26652, 0.32768, 0.76412},
        {0.09765625, 0.26816, 0.33825, 0.78050}, {0.10156250, 0.26967, 0.34878, 0.79631},
        {0.10546875, 0.27103, 0.35926, 0.81156}, {0.10937500, 0.27226, 0.36970, 0.82624},
        {0.11328125, 0.27334, 0.38008, 0.84037}, {0.11718750, 0.27429, 0.39043, 0.85393},
        {0.12109375, 0.27509, 0.40072, 0.86692}, {0.12500000, 0.27576, 0.41097, 0.87936},
        {0.12890625, 0.27628, 0.42118, 0.89123}, {0.13281250, 0.27667, 0.43134, 0.90254},
        {0.13671875, 0.27691, 0.44145, 0.91328}, {0.14062500, 0.27701, 0.45152, 0.92347},
        {0.14453125, 0.27698, 0.46153, 0.93309}, {0.14843750, 0.27680, 0.47151, 0.94214},
        {0.15234375, 0.27648, 0.48144, 0.95064}, {0.15625000, 0.27603, 0.49132, 0.95857},
        {0.16015625, 0.27543, 0.50115, 0.96594}, {0.16406250, 0.27469, 0.51094, 0.97275},
        {0.16796875, 0.27381, 0.52069, 0.97899}, {0.17187500, 0.27273, 0.53040, 0.98461},
        {0.17578125, 0.27106, 0.54015, 0.98930}, {0.17968750, 0.26878, 0.54995, 0.99303},
        {0.18359375, 0.26592, 0.55979, 0.99583}, {0.18750000, 0.26252, 0.56967, 0.99773},
        {0.19140625, 0.25862, 0.57958, 0.99876}, {0.19531250, 0.25425, 0.58950, 0.99896},
        {0.19921875, 0.24946, 0.59943, 0.99835}, {0.20312500, 0.24427, 0.60937, 0.99697},
        {0.20703125, 0.23874, 0.61931, 0.99485}, {0.21093750, 0.23288, 0.62923, 0.99202},
        {0.21484375, 0.22676, 0.63913, 0.98851}, {0.21875000, 0.22039, 0.64901, 0.98436},
        {0.22265625, 0.21382, 0.65886, 0.97959}, {0.22656250, 0.20708, 0.66866, 0.97423},
        {0.23046875, 0.20021, 0.67842, 0.96833}, {0.23437500, 0.19326, 0.68812, 0.96190},
        {0.23828125, 0.18625, 0.69775, 0.95498}, {0.24218750, 0.17923, 0.70732, 0.94761},
        {0.24609375, 0.17223, 0.71680, 0.93981}, {0.25000000, 0.16529, 0.72620, 0.93161},
        {0.25390625, 0.15844, 0.73551, 0.92305}, {0.25781250, 0.15173, 0.74472, 0.91416},
        {0.26171875, 0.14519, 0.75381, 0.90496}, {0.26562500, 0.13886, 0.76279, 0.89550},
        {0.26953125, 0.13278, 0.77165, 0.88580}, {0.27343750, 0.12698, 0.78037, 0.87590},
        {0.27734375, 0.12151, 0.78896, 0.86581}, {0.28125000, 0.11639, 0.79740, 0.85559},
        {0.28515625, 0.11167, 0.80569, 0.84525}, {0.28906250, 0.10738, 0.81381, 0.83484},
        {0.29296875, 0.10357, 0.82177, 0.82437}, {0.29687500, 0.10026, 0.82955, 0.81389},
        {0.30078125, 0.09750, 0.83714, 0.80342}, {0.30468750, 0.09532, 0.84455, 0.79299},
        {0.30859375, 0.09377, 0.85175, 0.78264}, {0.31250000, 0.09287, 0.85875, 0.77240},
        {0.31640625, 0.09267, 0.86554, 0.76230}, {0.32031250, 0.09320, 0.87211, 0.75237},
        {0.32421875, 0.09451, 0.87844, 0.74265}, {0.32812500, 0.09662, 0.88454, 0.73316},
        {0.33203125, 0.09958, 0.89040, 0.72393}, {0.33593750, 0.10342, 0.89600, 0.71500},
        {0.33984375, 0.10815, 0.90142, 0.70599}, {0.34375000, 0.11374, 0.90673, 0.69651},
        {0.34765625, 0.12014, 0.91193, 0.68660}, {0.35156250, 0.12733, 0.91701, 0.67627},
        {0.35546875, 0.13526, 0.92197, 0.66556}, {0.35937500, 0.14391, 0.92680, 0.65448},
        {0.36328125, 0.15323, 0.93151, 0.64308}, {0.36718750, 0.16319, 0.93609, 0.63137},
        {0.37109375, 0.17377, 0.94053, 0.61938}, {0.37500000, 0.18491, 0.94484, 0.60713},
        {0.37890625, 0.19659, 0.94901, 0.59466}, {0.38281250, 0.20877, 0.95304, 0.58199},
        {0.38671875, 0.22142, 0.95692, 0.56914}, {0.39062500, 0.23449, 0.96065, 0.55614},
        {0.39453125, 0.24797, 0.96423, 0.54303}, {0.39843750, 0.26180, 0.96765, 0.52981},
        {0.40234375, 0.27597, 0.97092, 0.51653}, {0.40625000, 0.29042, 0.97403, 0.50321},
        {0.41015625, 0.30513, 0.97697, 0.48987}, {0.41406250, 0.32006, 0.97974, 0.47654},
        {0.41796875, 0.33517, 0.98234, 0.46325}, {0.42187500, 0.35043, 0.98477, 0.45002},
        {0.42578125, 0.36581, 0.98702, 0.43688}, {0.42968750, 0.38127, 0.98909, 0.42386},
        {0.43359375, 0.39678, 0.99098, 0.41098}, {0.43750000, 0.41229, 0.99268, 0.39826},
        {0.44140625, 0.42778, 0.99419, 0.38575}, {0.44531250, 0.44321, 0.99551, 0.37345},
        {0.44921875, 0.45854, 0.99663, 0.36140}, {0.45312500, 0.47375, 0.99755, 0.34963},
        {0.45703125, 0.48879, 0.99828, 0.33816}, {0.46093750, 0.50362, 0.99879, 0.32701},
        {0.46484375, 0.51822, 0.99910, 0.31622}, {0.46875000, 0.53255, 0.99919, 0.30581},
        {0.47265625, 0.54658, 0.99907, 0.29581}, {0.47656250, 0.56026, 0.99873, 0.28623},
        {0.48046875, 0.57357, 0.99817, 0.27712}, {0.48437500, 0.58646, 0.99739, 0.26849},
        {0.48828125, 0.59891, 0.99638, 0.26038}, {0.49218750, 0.61088, 0.99514, 0.25280},
        {0.49609375, 0.62233, 0.99366, 0.24579}, {0.50000000, 0.63323, 0.99195, 0.23937},
        {0.50390625, 0.64362, 0.98999, 0.23356}, {0.50781250, 0.65394, 0.98775, 0.22835},
        {0.51171875, 0.66428, 0.98524, 0.22370}, {0.51562500, 0.67462, 0.98246, 0.21960},
        {0.51953125, 0.68494, 0.97941, 0.21602}, {0.52343750, 0.69525, 0.97610, 0.21294},
        {0.52734375, 0.70553, 0.97255, 0.21032}, {0.53125000, 0.71577, 0.96875, 0.20815},
        {0.53515625, 0.72596, 0.96470, 0.20640}, {0.53906250, 0.73610, 0.96043, 0.20504},
        {0.54296875, 0.74617, 0.95593, 0.20406}, {0.54687500, 0.75617, 0.95121, 0.20343},
        {0.55078125, 0.76608, 0.94627, 0.20311}, {0.55468750, 0.77591, 0.94113, 0.20310},
        {0.55859375, 0.78563, 0.93579, 0.20336}, {0.56250000, 0.79524, 0.93025, 0.20386},
        {0.56640625, 0.80473, 0.92452, 0.20459}, {0.57031250, 0.81410, 0.91861, 0.20552},
        {0.57421875, 0.82333, 0.91253, 0.20663}, {0.57812500, 0.83241, 0.90627, 0.20788},
        {0.58203125, 0.84133, 0.89986, 0.20926}, {0.58593750, 0.85010, 0.89328, 0.21074},
        {0.58984375, 0.85868, 0.88655, 0.21230}, {0.59375000, 0.86709, 0.87968, 0.21391},
        {0.59765625, 0.87530, 0.87267, 0.21555}, {0.60156250, 0.88331, 0.86553, 0.21719},
        {0.60546875, 0.89112, 0.85826, 0.21880}, {0.60937500, 0.89870, 0.85087, 0.22038},
        {0.61328125, 0.90605, 0.84337, 0.22188}, {0.61718750, 0.91317, 0.83576, 0.22328},
        {0.62109375, 0.92004, 0.82806, 0.22456}, {0.62500000, 0.92666, 0.82025, 0.22570},
        {0.62890625, 0.93301, 0.81236, 0.22667}, {0.63281250, 0.93909, 0.80439, 0.22744},
        {0.63671875, 0.94489, 0.79634, 0.22800}, {0.64062500, 0.95039, 0.78823, 0.22831},
        {0.64453125, 0.95560, 0.78005, 0.22836}, {0.64843750, 0.96049, 0.77181, 0.22811},
        {0.65234375, 0.96507, 0.76352, 0.22754}, {0.65625000, 0.96931, 0.75519, 0.22663},
        {0.66015625, 0.97323, 0.74682, 0.22536}, {0.66406250, 0.97679, 0.73842, 0.22369},
        {0.66796875, 0.98000, 0.73000, 0.22161}, {0.67187500, 0.98289, 0.72140, 0.21918},
        {0.67578125, 0.98549, 0.71250, 0.21650}, {0.67968750, 0.98781, 0.70330, 0.21358},
        {0.68359375, 0.98986, 0.69382, 0.21043}, {0.68750000, 0.99163, 0.68408, 0.20706},
        {0.69140625, 0.99314, 0.67408, 0.20348}, {0.69531250, 0.99438, 0.66386, 0.19971},
        {0.69921875, 0.99535, 0.65341, 0.19577}, {0.70312500, 0.99607, 0.64277, 0.19165},
        {0.70703125, 0.99654, 0.63193, 0.18738}, {0.71093750, 0.99675, 0.62093, 0.18297},
        {0.71484375, 0.99672, 0.60977, 0.17842}, {0.71875000, 0.99644, 0.59846, 0.17376},
        {0.72265625, 0.99593, 0.58703, 0.16899}, {0.72656250, 0.99517, 0.57549, 0.16412},
        {0.73046875, 0.99419, 0.56386, 0.15918}, {0.73437500, 0.99297, 0.55214, 0.15417},
        {0.73828125, 0.99153, 0.54036, 0.14910}, {0.74218750, 0.98987, 0.52854, 0.14398},
        {0.74609375, 0.98799, 0.51667, 0.13883}, {0.75000000, 0.98590, 0.50479, 0.13367},
        {0.75390625, 0.98360, 0.49291, 0.12849}, {0.75781250, 0.98108, 0.48104, 0.12332},
        {0.76171875, 0.97837, 0.46920, 0.11817}, {0.76562500, 0.97545, 0.45740, 0.11305},
        {0.76953125, 0.97234, 0.44565, 0.10797}, {0.77343750, 0.96904, 0.43399, 0.10294},
        {0.77734375, 0.96555, 0.42241, 0.09798}, {0.78125000, 0.96187, 0.41093, 0.09310},
        {0.78515625, 0.95801, 0.39958, 0.08831}, {0.78906250, 0.95398, 0.38836, 0.08362},
        {0.79296875, 0.94977, 0.37729, 0.07905}, {0.79687500, 0.94538, 0.36638, 0.07461},
        {0.80078125, 0.94084, 0.35566, 0.07031}, {0.80468750, 0.93612, 0.34513, 0.06616},
        {0.80859375, 0.93125, 0.33482, 0.06218}, {0.81250000, 0.92623, 0.32473, 0.05837},
        {0.81640625, 0.92105, 0.31489, 0.05475}, {0.82031250, 0.91572, 0.30530, 0.05134},
        {0.82421875, 0.91024, 0.29599, 0.04814}, {0.82812500, 0.90463, 0.28696, 0.04516},
        {0.83203125, 0.89888, 0.27824, 0.04243}, {0.83593750, 0.89298, 0.26981, 0.03993},
        {0.83984375, 0.88691, 0.26152, 0.03753}, {0.84375000, 0.88066, 0.25334, 0.03521},
        {0.84765625, 0.87422, 0.24526, 0.03297}, {0.85156250, 0.86760, 0.23730, 0.03082},
        {0.85546875, 0.86079, 0.22945, 0.02875}, {0.85937500, 0.85380, 0.22170, 0.02677},
        {0.86328125, 0.84662, 0.21407, 0.02487}, {0.86718750, 0.83926, 0.20654, 0.02305},
        {0.87109375, 0.83172, 0.19912, 0.02131}, {0.87500000, 0.82399, 0.19182, 0.01966},
        {0.87890625, 0.81608, 0.18462, 0.01809}, {0.88281250, 0.80799, 0.17753, 0.01660},
        {0.88671875, 0.79971, 0.17055, 0.01520}, {0.89062500, 0.79125, 0.16368, 0.01387},
        {0.89453125, 0.78260, 0.15693, 0.01264}, {0.89843750, 0.77377, 0.15028, 0.01148},
        {0.90234375, 0.76476, 0.14374, 0.01041}, {0.90625000, 0.75556, 0.13731, 0.00942},
        {0.91015625, 0.74617, 0.13098, 0.00851}, {0.91406250, 0.73661, 0.12477, 0.00769},
        {0.91796875, 0.72686, 0.11867, 0.00695}, {0.92187500, 0.71692, 0.11268, 0.00629},
        {0.92578125, 0.70680, 0.10680, 0.00571}, {0.92968750, 0.69650, 0.10102, 0.00522},
        {0.93359375, 0.68602, 0.09536, 0.00481}, {0.93750000, 0.67535, 0.08980, 0.00449},
        {0.94140625, 0.66449, 0.08436, 0.00424}, {0.94531250, 0.65345, 0.07902, 0.00408},
        {0.94921875, 0.64223, 0.07380, 0.00401}, {0.95312500, 0.63082, 0.06868, 0.00401},
        {0.95703125, 0.61923, 0.06367, 0.00410}, {0.96093750, 0.60746, 0.05878, 0.00427},
        {0.96484375, 0.59550, 0.05399, 0.00453}, {0.96875000, 0.58336, 0.04931, 0.00486},
        {0.97265625, 0.57103, 0.04474, 0.00529}, {0.97656250, 0.55852, 0.04028, 0.00579},
        {0.98046875, 0.54583, 0.03593, 0.00638}, {0.98437500, 0.53295, 0.03169, 0.00705},
        {0.98828125, 0.51989, 0.02756, 0.00780}, {0.99218750, 0.50664, 0.02354, 0.00863},
        {0.99609375, 0.49321, 0.01963, 0.00955}, {1.00000000, 0.47960, 0.01583, 0.01055}};

    static const Entry VIRIDIS[32] = {  //
        {0.0, 0.2823645529290169, 0.0, 0.3310101940118055},
        {0.03225806451612903, 0.29722632020441175, 0.027183413963800338, 0.37864029621447814},
        {0.06451612903225806, 0.3072803803150455, 0.0802447375229367, 0.4212673579883743},
        {0.0967741935483871, 0.3123264966480157, 0.12584534969096695, 0.457985073737091},
        {0.12903225806451613, 0.31237425118765544, 0.16879778927302974, 0.48815673654998043},
        {0.16129032258064516, 0.3077875785131497, 0.2101981170027718, 0.511619450888943},
        {0.1935483870967742, 0.29933085741427456, 0.25010033561427364, 0.5288208146468127},
        {0.22580645161290322, 0.2880586895294369, 0.28835583248860613, 0.5407177619131058},
        {0.25806451612903225, 0.2750965242322183, 0.3248913223445997, 0.5485120229082456},
        {0.29032258064516125, 0.2614262440352356, 0.3597968855678623, 0.5533850614666772},
        {0.3225806451612903, 0.2477338952312153, 0.39328759309143785, 0.5563075495959575},
        {0.3548387096774194, 0.23426522372462222, 0.4256765238335555, 0.5579283364248078},
        {0.3870967741935484, 0.2208774366275068, 0.4572924484791127, 0.5585488414824472},
        {0.4193548387096774, 0.2071151706305011, 0.48844291588547933, 0.5581501761803592},
        {0.45161290322580644, 0.19238993131530202, 0.5193782701027633, 0.5564329716762256},
        {0.4838709677419355, 0.17626507220153706, 0.5502647724624314, 0.5529002630158235},
        {0.5161290322580645, 0.1589045039992837, 0.5811708881961466, 0.5469358434427122},
        {0.5483870967741935, 0.14180310141349906, 0.6120609032437101, 0.537882137387205},
        {0.5806451612903225, 0.1289290768789858, 0.6428125092789465, 0.5251057926660444},
        {0.6129032258064516, 0.12737844015895444, 0.6732171143800528, 0.5080488847567046},
        {0.6451612903225806, 0.1441804301921067, 0.7030136844759319, 0.4862684965591734},
        {0.6774193548387096, 0.1805504147795526, 0.7318951275563422, 0.4594108399264241},
        {0.7096774193548387, 0.23246629839421223, 0.7595227413720251, 0.4272148754878393},
        {0.7419354838709677, 0.2955966115638969, 0.7855342571397639, 0.3894744537385309},
        {0.7741935483870968, 0.36690903818194603, 0.8095679467648405, 0.3460474232378068},
        {0.8064516129032258, 0.44452103469874904, 0.8312792890501436, 0.29679122529015456},
        {0.8387096774193548, 0.5270107870979416, 0.8503789777911802, 0.24157408056415985},
        {0.8709677419354839, 0.6129677032098788, 0.8667148025192438, 0.18037880054649338},
        {0.9032258064516129, 0.7006476041657969, 0.8804041088681586, 0.114029402487933},
        {0.9354838709677419, 0.7879194958353214, 0.8919655007329689, 0.050317962262272926},
        {0.967741935483871, 0.8726031817793708, 0.9023098719403728, 0.043816043656816815},
        {1.0, 0.9529994532916154, 0.9125452328290099, 0.11085876909361342}};

    static const Entry COOLWARM[32] = {  //
        {0.0, 0.334790850135, 0.283084370265, 0.756495219864},
        {0.0322580645161, 0.369439113767, 0.340869393976, 0.805750658221},
        {0.0645161290323, 0.404259141398, 0.397361902553, 0.850387531691},
        {0.0967741935484, 0.439372341987, 0.452371719752, 0.889932656966},
        {0.129032258065, 0.474847206273, 0.505583761378, 0.923972751068},
        {0.161290322581, 0.510694089363, 0.556623493426, 0.952158409473},
        {0.193548387097, 0.546862458948, 0.605090005961, 0.974207286728},
        {0.225806451613, 0.583240973466, 0.650575046912, 0.989906418117},
        {0.258064516129, 0.619660243289, 0.692675288486, 0.999113641233},
        {0.290322580645, 0.655897812481, 0.731001029682, 1.00175809405},
        {0.322580645161, 0.691684753183, 0.765182858783, 0.997839781888},
        {0.354838709677, 0.72671325195, 0.794877041512, 0.98742821945},
        {0.387096774194, 0.760644641819, 0.819770032611, 0.97066016544},
        {0.41935483871, 0.793117451944, 0.839582318687, 0.947736475702},
        {0.451612903226, 0.823755174238, 0.854071696238, 0.918918104643},
        {0.483870967742, 0.852173561779, 0.863036027811, 0.884521282118},
        {0.516129032258, 0.880463496336, 0.857749410743, 0.842568817483},
        {0.548387096774, 0.906421764642, 0.838303054478, 0.79544240184},
        {0.58064516129, 0.926833712286, 0.813464967809, 0.74663927939},
        {0.612903225806, 0.94162851597, 0.783445472637, 0.696659885635},
        {0.645161290323, 0.950755018966, 0.74848506033, 0.645987903113},
        {0.677419354839, 0.954187930347, 0.708846934849, 0.595085813436},
        {0.709677419355, 0.951932437454, 0.664806956566, 0.544391069305},
        {0.741935483871, 0.944027639826, 0.6166393937, 0.494312932356},
        {0.774193548387, 0.930549054574, 0.5645954156, 0.445230028189},
        {0.806451612903, 0.911610347414, 0.508868029202, 0.397488683954},
        {0.838709677419, 0.887364384266, 0.449529386408, 0.351402140488},
        {0.870967741935, 0.858003661237, 0.38640533116, 0.307250775909},
        {0.903225806452, 0.823760147778, 0.318784841899, 0.265283548281},
        {0.935483870968, 0.784904563871, 0.244588073239, 0.225720968845},
        {0.967741935484, 0.741745104748, 0.156900671237, 0.188760053446},
        {1.0, 0.694625624821, 0.00296461045768, 0.154581828278}};

    static const Entry EXT_KINDLMANN[32] = {  //
        {0.0, 0.0, 0.0, 0.0},
        {0.0322580645161, 0.106334390406, 0.00567696995302, 0.118928794028},
        {0.0645161290323, 0.138241070858, 0.0112148994321, 0.235071288925},
        {0.0967741935484, 0.152795841881, 0.0158847736303, 0.331892934621},
        {0.129032258065, 0.173294928294, 0.0198237892539, 0.41530977414},
        {0.161290322581, 0.0272049543726, 0.0596075996998, 0.570619892175},
        {0.193548387097, 0.0196640005344, 0.180060306624, 0.409245107267},
        {0.225806451613, 0.0148460002315, 0.233425924062, 0.306551326775},
        {0.258064516129, 0.0130388007803, 0.273099143181, 0.251893872221},
        {0.290322580645, 0.0150271863754, 0.310268512831, 0.20088396568},
        {0.322580645161, 0.0165401628187, 0.346774181184, 0.139759781303},
        {0.354838709677, 0.0183734964025, 0.382549168808, 0.0740383250959},
        {0.387096774194, 0.020105867968, 0.417624209872, 0.0458016594506},
        {0.41935483871, 0.112949184552, 0.449043877823, 0.0216911860571},
        {0.451612903226, 0.255512576861, 0.470596387485, 0.0225457320031},
        {0.483870967742, 0.393567950939, 0.482888932617, 0.0230903421353},
        {0.516129032258, 0.532667312833, 0.484366968635, 0.0256883476979},
        {0.548387096774, 0.71263777937, 0.4526085566, 0.0343042390523},
        {0.58064516129, 0.930163139514, 0.348313690817, 0.0443691755525},
        {0.612903225806, 0.964356020699, 0.384731208916, 0.253529518557},
        {0.645161290323, 0.970905191634, 0.443791186749, 0.389270442758},
        {0.677419354839, 0.97564056562, 0.490577794622, 0.589948544912},
        {0.709677419355, 0.977766277914, 0.533280168858, 0.769439233062},
        {0.741935483871, 0.979713323691, 0.575158779329, 0.915153328516},
        {0.774193548387, 0.940912898082, 0.647609954809, 0.983152890267},
        {0.806451612903, 0.896532736211, 0.723651057828, 0.986762599008},
        {0.838709677419, 0.891324828761, 0.778840494007, 0.98941946091},
        {0.870967741935, 0.907619533382, 0.825097470293, 0.991608048412},
        {0.903225806452, 0.920016273873, 0.872383038081, 0.993919549},
        {0.935483870968, 0.923426442754, 0.922179806515, 0.996257950289},
        {0.967741935484, 0.937333894324, 0.968496280069, 0.99700762872},
        {1.0, 1.0, 1.0, 1.0}};

    static const Entry PLASMA[32] = {  //
        {0.0, 0.18500126283629117, 0.0, 0.5300734481832133},
        {0.03225806451612903, 0.23280798531292202, 0.0, 0.5663460194355492},
        {0.06451612903225806, 0.27774403335700476, 0.0, 0.5940887229557431},
        {0.0967741935483871, 0.3218984040469345, 0.0, 0.6171014162170981},
        {0.12903225806451613, 0.3658460567574231, 0.0, 0.636106326030986},
        {0.16129032258064516, 0.4096400169901105, 0.0, 0.650704976950701},
        {0.1935483870967742, 0.4530865724513315, 0.0, 0.6600372359929895},
        {0.22580645161290322, 0.4958586756265672, 0.0, 0.663149473546627},
        {0.25806451612903225, 0.5375597495484363, 0.0, 0.6592763803680873},
        {0.29032258064516125, 0.5777886526487028, 0.0, 0.6481410668005011},
        {0.3225806451612903, 0.6161967526140943, 0.00946866288733594, 0.6301724702376648},
        {0.3548387096774194, 0.6525546984194616, 0.07543175641148503, 0.6065793828400363},
        {0.3870967741935484, 0.6867862223412597, 0.12613396593176182, 0.5790515347394288},
        {0.4193548387096774, 0.718955619210146, 0.1705479391890951, 0.5493545554493838},
        {0.45161290322580644, 0.7492241108506705, 0.212031951485315, 0.5189317181862739},
        {0.4838709677419355, 0.777778289424557, 0.2518751706682369, 0.4887169189279263},
        {0.5161290322580645, 0.8047803831958162, 0.29081060055294533, 0.4591371242517161},
        {0.5483870967741935, 0.8303190755961327, 0.32935690914477145, 0.43024942250093773},
        {0.5806451612903225, 0.8543927145113053, 0.36794575948702757, 0.40188627740427024},
        {0.6129032258064516, 0.8768902251837156, 0.40695681284058494, 0.3737868866989539},
        {0.6451612903225806, 0.8976060527068899, 0.44672671303614025, 0.3456751918378199},
        {0.6774193548387096, 0.9162458538360101, 0.4875549161009913, 0.3173242042796378},
        {0.7096774193548387, 0.932448320984704, 0.5296932053152419, 0.28857262860312527},
        {0.7419354838709677, 0.9458023759154403, 0.5733398475187904, 0.2593470428132603},
        {0.7741935483870968, 0.9558753017301963, 0.6186214159649234, 0.22966238980267967},
        {0.8064516129032258, 0.9622023766783513, 0.6656228582548607, 0.19973953961986202},
        {0.8387096774193548, 0.964273782059227, 0.7143819654513636, 0.17030518048909926},
        {0.8709677419354839, 0.9615161888612608, 0.7649102466340377, 0.14325126737523788},
        {0.9032258064516129, 0.9532892220744008, 0.817181923297583, 0.12263378614142162},
        {0.9354838709677419, 0.938984382355116, 0.8710941994844004, 0.1143137135904449},
        {0.967741935483871, 0.9182680324484542, 0.9264392704144366, 0.11791750492313907},
        {1.0, 0.894058310302958, 0.9822535793047805, 0.0810687655704728}};

    double remap(double a1, double a2, double av, double b1, double b2)
    {
        double dat = fabs(a2 - a1);
        double dav = fabs(av - a1);

        double dbt = fabs(b2 - b1);
        double dbv = dbt * (dav / dat);

        double bv = b1 + dbv;

        return bv;
    }
    void colormap(double s, double &r, double &g, double &b, unsigned int size, const Entry map[])
    {
        for (unsigned int i = 1; i < size; ++i)
        {
            const Entry &p = map[i - 1];
            const Entry &n = map[i];

            if (s < n.s)
            {
                r = remap(p.s, n.s, s, p.r, n.r);
                g = remap(p.s, n.s, s, p.g, n.g);
                b = remap(p.s, n.s, s, p.b, n.b);
                return;
            }
        }

        r = map[size - 1].r;
        g = map[size - 1].g;
        b = map[size - 1].b;
    }
}  // namespace

void robowflex::color::viridis(double s, double &r, double &g, double &b)
{
    colormap(s, r, g, b, 32, VIRIDIS);
}

void robowflex::color::coolwarm(double s, double &r, double &g, double &b)
{
    colormap(s, r, g, b, 32, COOLWARM);
}

void robowflex::color::extKindlmann(double s, double &r, double &g, double &b)
{
    colormap(s, r, g, b, 32, EXT_KINDLMANN);
}

void robowflex::color::plasma(double s, double &r, double &g, double &b)
{
    colormap(s, r, g, b, 32, PLASMA);
}

void robowflex::color::turbo(double s, double &r, double &g, double &b)
{
    colormap(s, r, g, b, 256, TURBO);
}

void robowflex::color::grayscale(double s, double &r, double &g, double &b)
{
    r = s;
    g = s;
    b = s;
}

void robowflex::color::toGrayscale(double &r, double &g, double &b)
{
    double v = 0.3 * r + 0.59 * g + 0.11 * b;
    r = g = b = v;
}

void robowflex::color::rgb2hsv(double r, double g, double b, double &h, double &s, double &v)
{
    double min, max, delta;

    min = r < g ? r : g;
    min = min < b ? min : b;

    max = r > g ? r : g;
    max = max > b ? max : b;

    v = max;
    delta = max - min;
    if (delta < 0.00001)
    {
        s = 0;
        h = 0;
        return;
    }
    if (max > 0.0)
        s = delta / max;
    else
    {
        s = 0.0;
        h = constants::nan;
        return;
    }
    if (r >= max)
        h = (g - b) / delta;  // between yellow & magenta
    else if (g >= max)
        h = 2.0 + (b - r) / delta;  // between cyan & yellow
    else
        h = 4.0 + (r - g) / delta;  // between magenta & cyan

    h *= 60.0;  // degrees
    if (h < 0.0)
        h += 360.0;

    h = TF::toRadians(h);
}

void robowflex::color::hsv2rgb(double h, double s, double v, double &r, double &g, double &b)
{
    double hh, p, q, t, ff;
    long i;

    if (s <= 0.0)
    {
        r = v;
        g = v;
        b = v;
        return;
    }

    hh = TF::toDegrees(h);
    if (hh < 0.0)
        hh += 360.0;
    else if (hh >= 360.0)
        hh = 0.0;

    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = v * (1.0 - s);
    q = v * (1.0 - (s * ff));
    t = v * (1.0 - (s * (1.0 - ff)));

    switch (i)
    {
        case 0:
            r = v;
            g = t;
            b = p;
            break;
        case 1:
            r = q;
            g = v;
            b = p;
            break;
        case 2:
            r = p;
            g = v;
            b = t;
            break;
        case 3:
            r = p;
            g = q;
            b = v;
            break;
        case 4:
            r = t;
            g = p;
            b = v;
            break;
        case 5:
        default:
            r = v;
            g = p;
            b = q;
            break;
    }
}
