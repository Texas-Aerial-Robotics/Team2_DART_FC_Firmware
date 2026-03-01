/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "mpu9250.h"
#include "bmp388.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern IMU_ProcessedData_t imu_processed_data;
extern IMU_Angles_t imu_angles;
extern Kalman_t KalmanPitch;
extern Kalman_t KalmanRoll;
extern BMP388_ProcessedData_t bmp388_processedData;
extern BMP388_RawData_t bmp388_rawData;
volatile double previous_time = 0;

volatile uint8_t timer_flag = 0;
volatile uint8_t update_ccr = 0;
double control_angles[] = {
0.000000000000000000e+00,
-2.482147825034401387e-02,
-2.648524798658543483e-02,
-1.120426764553852372e-01,
-1.476462950213445202e-01,
-1.246013626386344164e-01,
-1.566571422534693281e-01,
5.733114250277299151e-02,
2.984923399793853321e-02,
5.335547354298371403e-02,
-4.560851647698886159e-03,
-1.406117319223361591e-02,
8.980653486098621324e-03,
3.260883964839043914e-02,
-7.196527326023787266e-03,
4.230621307544735199e-02,
1.058229171094886012e-01,
-4.656192033505328343e-02,
-9.970729609722871201e-02,
1.500208153619158810e-02,
-8.033287249149695386e-02,
6.546133699745473877e-02,
-6.014893786541153364e-02,
8.273271305708157497e-02,
3.711918856024850472e-02,
5.329818889721076383e-03,
7.908945423398897057e-02,
-5.311449294668754012e-02,
1.441061870178089455e-01,
-3.268243471925414562e-02,
7.721295169968138417e-02,
4.821097490235138477e-02,
1.394498019296377123e-01,
7.369941974351267022e-02,
2.993573408995214152e-02,
7.952228083697129857e-02,
5.201255710789443043e-03,
-1.024642129448632699e-01,
-1.055500194166685696e-01,
-1.535524257547723169e-02,
3.364963011004780402e-02,
1.886875903452011217e-03,
-2.793582136545309097e-02,
1.234783514211000577e-01,
-3.263687668406888009e-02,
4.930510238303603687e-02,
7.169084516932232309e-02,
-1.266895202951433784e-02,
4.189084413439076587e-02,
6.258474916938124022e-02,
3.254256436182279694e-02,
1.139582049966351290e-02,
-8.019247398909069846e-02,
-8.786732822407467003e-02,
-1.124258703005491383e-01,
-5.123662834542867112e-02,
1.193671180359743011e-01,
3.300955855743308714e-02,
3.721121126966873277e-02,
3.508027452422906450e-02,
4.694576904873391104e-02,
5.056647631106732897e-03,
6.093219291628947099e-02,
4.279040973286086721e-03,
7.484002904408467403e-03,
-7.333184990149603510e-02,
-1.149262317592882193e-01,
-5.054316539911127976e-02,
-1.649684206785921722e-03,
-4.713755029571682631e-02,
5.707259814159416217e-02,
-7.884827447796392830e-02,
-6.904609575013451184e-02,
5.705143750242312739e-02,
-8.398536300702148794e-03,
-2.530836251849590685e-02,
2.225147896694133071e-02,
-1.139672459254822100e-02,
3.899969578281682180e-03,
-6.939246413723117873e-03,
2.140148983675973712e-02,
1.339703845561722624e-02,
7.149047397320369701e-03,
-2.803142221355533082e-03,
1.320880048840797870e-02,
-8.911708307658496725e-03,
-2.002574866167207771e-02,
-2.777809006771696579e-02,
1.561378223707168486e-02,
1.488936779450883430e-02,
3.521162450842715581e-03,
-1.898665096811142164e-02,
-5.728812736724767790e-03,
2.196018497722466642e-02,
-2.456399583214773953e-03,
-1.320773052615967572e-02,
4.435059624595109884e-03,
8.299577311851926917e-03,
-7.432819812379647291e-03,
5.031521140436988164e-03,
7.461030424137693288e-03,
-4.826713845855950223e-03,
-9.857590831438146906e-03,
-2.367287272334460749e-02,
2.065841122337197175e-02,
-3.378817244573770195e-03,
-1.455363477332420954e-02,
1.136787699325087463e-02,
-1.591482206720702328e-02,
1.021409112049628289e-02,
1.006742600409814108e-02,
-3.247652199491056491e-02,
2.927108617746970384e-02,
-7.825644424876555469e-03,
2.172510934979360667e-02,
-4.359667534010226893e-03,
2.496813880291355592e-02,
-2.726209994914427504e-02,
1.163323341348613225e-02,
-7.189220501159475103e-04,
-1.464127653255048109e-02,
8.110207054061758028e-03,
2.220191179997902178e-02,
6.682944650060263121e-03,
-2.344247469429550437e-02,
2.033466443647121216e-02,
6.303062145570050706e-03,
-8.091361921464373594e-03,
-1.747590263950807993e-02,
-5.928218536305565785e-03,
-7.724521346364251334e-03,
-5.938834929033477358e-03,
-1.228021090690335770e-02,
1.237609794052295462e-03,
-1.514089599765749798e-02,
-5.934960649636741511e-03,
8.436259552121593008e-03,
-9.180946609157608629e-03,
-1.586178909639716222e-02,
1.438682659988022201e-02,
-3.995662842836083563e-03,
-4.979095337502463525e-02,
1.343939474156289476e-02,
-4.461390799494996395e-02,
-5.277893190029249795e-03,
1.325761021089921603e-02,
1.729460447746945101e-02,
2.268034711465946943e-02,
2.066818317654051801e-02,
-2.735455328831210703e-02,
3.459964830940431748e-03,
-9.154868651267819221e-03,
-9.239782020532633089e-03,
1.603452453602313699e-02,
1.134195129807409955e-02,
3.485666828362277148e-02,
-1.072548328054875703e-02,
3.291201058708941030e-02,
2.680294619061176706e-02,
-8.267045280102396781e-03,
-1.654220028608932580e-02,
-5.669990720594383395e-03,
1.012044803988952114e-02,
1.676978699038500151e-02,
2.823528574106318537e-02,
-1.851652471926656940e-02,
2.595282476324834581e-02,
-4.574116053105091104e-03,
-3.123237884397554731e-02,
-3.359115070545096865e-02,
3.959166256905626187e-03,
-1.771355596035075960e-02,
8.932746147646069071e-03,
1.745403414730180638e-02,
-1.878814843421219719e-03,
7.758318636082865358e-03,
-1.568394182122461074e-03,
9.985563817400783596e-03,
-1.911517960191828208e-02,
7.666846842020637841e-03,
2.155675541672212939e-02,
8.364370265952407110e-03,
1.786521323726517063e-02,
2.508995623324425731e-02,
-2.966518587852799357e-02,
-3.439149498221159146e-02,
-4.223517224344575452e-03,
1.814544133357212999e-02,
-7.139043003226310143e-03,
1.696715926397191626e-02,
-2.171215688944715810e-02,
3.565818350194600861e-02,
4.869846684854605041e-03,
2.375155334529341182e-03,
-1.292464592082657321e-02,
-8.350704651386821678e-03,
4.743804046265941501e-03,
-2.382126909023754222e-02,
2.095128864183976383e-02,
6.006401367821162171e-03,
4.037973703451596535e-02,
6.596853003260678798e-03,
1.588732709566380514e-03,
-2.467081996282340789e-02,
6.949649979102303157e-03,
1.332207730115565932e-02,
6.070383065575627769e-03,
-1.472407997831486071e-02,
-6.869025183275945592e-03,
-3.780093866668245151e-02,
1.003964454411758293e-02,
-6.792964991725798131e-03,
3.185704754106962605e-02,
-1.441076042130417720e-02,
3.183330672969928582e-02,
3.889387739105078234e-02,
-1.456897693517535640e-02,
-3.022438687158618409e-02,
-5.513886052953140098e-03,
-1.568761536775572715e-02,
-1.763979900829828334e-02,
1.925657433392438320e-02,
-5.294467454879981522e-02,
2.450050239790364792e-02,
-3.571533684896670280e-02,
5.922024318908448819e-03,
3.170294133753081417e-02,
1.146634211277121687e-02,
9.134449446833661649e-04,
-3.141616515327944059e-02,
1.294667942178581027e-02,
3.255387252562391004e-02,
-4.082886136095835450e-02,
1.372258237756875067e-03,
5.183036975471885799e-02,
-8.082042085381340238e-03,
6.438939363839903812e-03,
-2.558742842252166583e-02,
4.255093155071033861e-03,
4.163698343637847743e-02,
1.427185415546828283e-02,
-9.881284831919912026e-03,
-2.269103483919492764e-02,
3.583965932864733445e-02,
-3.082583525717437749e-03,
-2.355935223337740983e-02,
3.542718768578515757e-03,
-2.012068153013398669e-02,
-4.309934628815470688e-03,
1.219177516417181348e-02,
-2.842528803688794403e-02,
-2.955473395390846927e-02,
-2.851915856967360968e-02,
-2.701147689940914740e-02,
-2.076980891757628997e-02,
1.646342990548000512e-02,
7.618072497689978850e-02,
-9.313432248524751086e-03,
-3.306029878577890213e-02,
-5.191930274188421429e-02,
-1.345204911033416259e-02,
1.433375699819421957e-02,
-6.190396477383899171e-02,
-3.534767029771659907e-02,
2.104331925935293279e-02,
4.875631778601958316e-02,
1.043071871265243561e-02,
5.887142568801499547e-03,
-4.063936819673634460e-02,
3.909861680917986471e-02,
-2.211684327626121591e-02,
-1.857293630163258519e-02,
1.918990298273815648e-02,
-4.680486621416699543e-03,
2.035180666075363590e-02,
1.212106182996888250e-02,
-2.810127273742597451e-02,
-1.178230939702297822e-02,
-9.499486188327339528e-03,
3.926604384575538981e-02,
1.776417111570082960e-02,
4.815196594155144588e-02,
-4.678830592364820878e-02,
1.352309555993708515e-02,
-1.577750838353380453e-02,
-1.874622904757539121e-02,
3.620185168748516619e-02,
1.041133842717036073e-02,
-3.138423497834531145e-02,
-1.593917448203598444e-02,
1.637909555885216917e-02,
6.299239439785156436e-02,
2.997566096617886033e-02,
-2.521014746259948869e-02,
-3.120786398246125273e-03,
1.101664539764275232e-02,
-1.395571289550025414e-02,
-9.061509477359577791e-03,
1.629756923618064929e-02,
3.051006467239168133e-03,
-1.386343210515567428e-02,
4.806881643884670235e-02,
4.904738404276899610e-02,
1.874401826241772762e-02,
1.259436207554556417e-02,
-4.757574274888967669e-02,
6.175282422254009355e-03,
5.137230922562011082e-02,
-3.935720027986863885e-03,
-3.739003670274045066e-02,
1.526719008117611263e-02,
-5.550133974674320483e-04,
-6.003124631503067854e-02,
-1.069752283113561964e-02,
6.803433979176315991e-04,
-1.088810243196968591e-02,
5.423754821228345124e-02,
-1.779623255416514643e-02,
-1.526817852725823564e-02,
2.701002257188549541e-02,
-5.371538938877891411e-02,
-3.482062058287415437e-03,
1.908527312522921918e-02,
4.934956450678389117e-02,
-8.260884675313301395e-03,
1.939153960114888345e-02,
-2.858639347882338880e-03,
-2.361345821326180788e-02,
-7.567182699722243243e-03,
1.375767723893804237e-02,
-7.634306525515210766e-03,
6.572866415801285889e-02,
3.454571455412360631e-02,
7.566951970594676391e-03,
1.694048067804303226e-02,
-3.396653717131509903e-02,
-1.025377430730192371e-02,
2.552853980945041582e-02,
-1.882261643545830431e-03,
-1.993849672831849232e-02,
-2.597553464650954533e-02,
-1.358665819306208514e-02,
-1.465846814170434532e-02,
-3.243121749269013698e-02,
-6.721219158128047266e-02,
5.535305994634457522e-02,
-1.121526647087211191e-02,
-6.940991318671727028e-02,
-3.462555732143973459e-03,
-1.296545367542039848e-02,
-3.431329804686429619e-02,
2.389321066875487592e-02,
-9.031199390530068524e-03,
-4.517631480262361554e-02,
1.220531994750106465e-02,
4.017851779843945731e-02,
-1.826688435062433224e-02,
-3.102230438300813964e-02,
2.184519156087012803e-02,
-3.442426432910077050e-02,
-2.902847086724495707e-02,
-8.763605355826570298e-03,
-7.286181330642846517e-03,
2.816172821357577471e-03,
2.884461269877148029e-02,
-4.926264548578052072e-02,
2.496621033224590350e-02,
2.727172072472048778e-02,
-5.733357506996893810e-03,
4.053187555332958547e-02,
1.055307598570619752e-02,
-3.121453005640921138e-02,
2.631524378645777162e-03,
2.135043084350693801e-02,
2.616218909940325882e-02,
-4.558328122754975835e-03,
-1.345424193854072780e-03,
3.967787783281075055e-04,
2.250353475949310247e-02,
4.102065576437405325e-02,
3.979462855737313925e-02,
-1.980992142946232798e-02,
2.582046880138159223e-02,
-2.617082598498616033e-02,
4.098594999115579007e-02,
-3.774045602215882200e-02,
2.818342385693845681e-02,
5.517571639300934694e-02,
3.620331569227091401e-02,
1.197124784501679695e-02,
2.047355264680081333e-02,
-4.535242437063463639e-02,
5.056573690352279826e-02,
-3.198649749811036114e-02,
-2.648624420438555963e-02,
-9.564103014271147646e-03,
-3.923696923410543297e-02,
-4.474582831491749624e-02,
3.014358092562324440e-02,
-1.444110160677664588e-02,
3.406647807310338866e-02,
-1.050360632082504055e-02,
5.633062484656196217e-02,
1.013693335863079595e-02,
-6.249174532086359049e-03,
8.911145202573541799e-03,
-3.895928323063929666e-02,
-3.124621582833812458e-02,
-2.095655386856246777e-02,
3.828826398420759943e-03,
-5.924532353267663752e-04,
1.100437945744263228e-02,
-4.052424356350962237e-02,
-5.041296294292492430e-02,
-4.110020930807817968e-02,
2.370934237388806765e-02,
3.997405891864286298e-02,
-6.671351685290365097e-03,
-2.606039218333739901e-02,
2.037464662889161560e-02,
2.137649135759986732e-02,
1.056213898598289819e-03,
-3.586401196327797941e-02,
-1.975184157079989955e-02,
-6.931499997672329987e-03,
-3.857559976802316798e-02,
2.716401311587949424e-02,
-6.958016424156128650e-03,
-1.411597790170354347e-02,
-1.015783165746796633e-02,
2.046797700030485861e-02,
-1.885652382573286406e-02,
-2.562156415635753401e-02,
9.767836111784739678e-03,
1.898967439051948980e-02,
1.845375822230512636e-02,
1.624611308895222012e-02,
-1.744257380863884974e-02,
-3.751685586516651655e-02,
3.507079774569801800e-02,
2.329551205962569066e-02,
-1.411694624650018486e-02,
-5.562556599602938448e-04,
1.745656121020536635e-03,
6.250691574910239266e-03,
-4.289732123998880875e-02,
3.429888083652247316e-02,
1.764347492152543162e-02,
6.519050026274557375e-03,
-3.034339342026025343e-02,
3.202123663663877795e-02,
1.510785696464560235e-02,
1.992563555689304614e-03,
3.530050582558207084e-02,
3.288350402348236539e-03,
7.907141362309281010e-04,
-2.681968266318105962e-02,
2.030486692710042895e-02,
1.939644060859520638e-02,
-1.629501818370797384e-02,
4.078584941072459918e-02,
3.113375554855550217e-02,
2.984497403238880325e-02,
-3.377435081101158587e-02,
4.192658878336331940e-02,
1.179100525946110041e-02,
-1.990623162150232869e-02,
3.933816184799690530e-02,
1.891017187616002640e-03,
-1.529680605693128047e-02,
-2.200791001054242663e-02,
1.135621793306183618e-02,
1.813971773783098346e-02,
2.176219849307557686e-02,
-3.798382244962685700e-02,
-4.847298534512376605e-02,
9.811515522369340311e-03,
-4.867679145367169447e-02,
2.299153607098668173e-02,
5.712953575088491193e-02,
1.138035647308593794e-02,
-1.030490310771841993e-02,
-4.428074840705606419e-03,
-4.889956549624532856e-02,
2.436586933580483019e-03,
-1.483870444541666826e-02,
-2.795188363192424932e-02,
2.193446268864229429e-02,
2.503144074643249162e-03,
-1.881667117463524386e-02,
-1.187535243655883817e-02,
7.821482192532810268e-03,
-5.662068279732321643e-03,
-1.580522134991558039e-02,
4.264097776641162518e-03,
3.166348362140855317e-03,
-9.445052776324383852e-05,
7.779450240277225670e-04,
9.259020823550220391e-04,
};
float control_angle_gain = 250.0f;
uint16_t duty1, duty2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
double get_dt();
uint16_t setDutyCH1(TIM_HandleTypeDef *htim, float angle);
uint16_t setDutyCH2(TIM_HandleTypeDef *htim, float angle);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);  // Enable TIM2 interrupt
  HAL_TIM_Base_Start_IT(&htim1);
  char buffer[40] = {'\0'};
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Enable TIM1 Channel 1 interrupt
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // Enable TIM1 Channel 1 interrupt
  mpu9250_setup();
  bmp388_setup();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //process IMU data on timer interrupt
	  if(timer_flag)
	  {
		  timer_flag = 0;	//reset timer flag

		  mpu9250_getProcessedAngle();
		  bmp388_getData();
	  }

	  //send data through UART
	  // snprintf(buffer, sizeof(buffer), "%.4f,%.4f,%.4f\n", imu_angles.pitch, imu_angles.roll, imu_angles.yaw);
	  // HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//	  duty1=setDutyCH1(&htim1, 0.0f);
//      duty2=setDutyCH2(&htim1, 0.0f);
//
//      HAL_Delay(1000);

//      FILE *file = fopen("/Users/rachitkakkar/Documents/root/u_controls_dt_0.02.txt", "r");
//      if (!file) {
//          // printf("Error opening file\n");
//          return 1;
//      }

//      double a, c, d;
//      while (fscanf(file, "%lf %lf %lf", &a, &c, &d) == 3) {
//    	  duty1=setDutyCH1(&htim1, c); // C is equal to pitch
//          HAL_Delay(20);
//      }
//
//      fclose(file);
//
    for (int i = 0; i < sizeof(control_angles); i++) {
    	float final_angle = control_angles[i] * control_angle_gain;
      duty1 = setDutyCH1(&htim1, final_angle);
      duty2 = setDutyCH2(&htim1, final_angle);
      HAL_Delay(1000);
    }
//	  duty1 = setDutyCH1(&htim1, -90.0f);
//	  duty2= setDutyCH2(&htim1, 90.0f);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 240-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 799;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin|SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI1_CS_Pin SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
double get_dt()
{
    double current_time = HAL_GetTick() / 1000.0;  // Get time in seconds
    double dt = current_time - previous_time;
    previous_time = current_time;  // Update for the next call
    return dt;
}

uint16_t setDutyCH1(TIM_HandleTypeDef *htim, float angle) {
    // Limit range from -90 to 90 degrees
    if (angle < -90.0f) angle = -90.0f;
    if (angle > 90.0f) angle = 90.0f;

    // Convert angle to a range between 0-180 degrees (just for testing purposes)
    float transformed_angle = angle + 90.0f;

    // Calculate the PWM pulse width in microseconds (1000 µs to 2000 µs; total period is 20000 µs)
    // Assume minimum pulse is 0.6ms and the maximum pulse is 2.4ms
    float pulseTime = (transformed_angle * 1800.0f) / 180.0f + 600.0f;

    // Convert pulse width in µs to CCR value
    uint16_t ccrValue = (uint16_t)(pulseTime);

    htim->Instance->CCR1 = ccrValue;
    return ccrValue;
}

uint16_t setDutyCH2(TIM_HandleTypeDef *htim, float angle) {
    // Limit range from -90 to 90 degrees
    if (angle < -90.0f) angle = -90.0f;
    if (angle > 90.0f) angle = 90.0f;

    // Convert angle to a range between 0-180 degrees
    float transformed_angle = angle + 90.0f;

    // Calculate the PWM pulse width in microseconds (1000 µs to 2000 µs; total period is 20000 µs)
    // Assume minimum pulse is 0.6ms and the maximum pulse is 2.4ms
    float pulseTime = (transformed_angle * 1800.0f) / 180.0f + 600.0f;

    // Convert pulse width in µs to CCR value
    uint16_t ccrValue = (uint16_t)(pulseTime);

    htim->Instance->CCR2 = ccrValue;
    return ccrValue;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2)
	{
		timer_flag = 1;
	}
	if (htim == &htim1)
	{
		update_ccr = 1;
	}
}
int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
