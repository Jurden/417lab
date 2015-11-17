// Jordan Millett

#include <iostream>     // Include the standard C++ IO library functions
#include <cstdlib>       // For exit() prototype
#include <iomanip>
#include <cmath>

#define pi 	3.14159265

using namespace std;

class Matrix41;

class Matrix44 {
	public:
	//private:
		float element[4][4];
	//public:
		Matrix44(void);
		Matrix44 transpose(void) const;
		friend istream& operator>> (istream& s, Matrix44& t);
		friend ostream& operator<< (ostream& s, const Matrix44& t);
		friend Matrix44 operator*(const Matrix44 &A, const Matrix44 &B); // Overriding * operator to multiply the matrices
		friend Matrix41 operator*(const Matrix44 &A, const Matrix41 &P); // Overriding * operator to multiply the 4x4 and 4x1
		Matrix44 inverse(void) const;
		friend Matrix44 FKinematics(float d, float O, float a, float A);
		friend void IKinematics(const Matrix44 &A);
};

Matrix44 operator*(const Matrix44 &A, const Matrix44 &B){	// Actualy overriding of *
	Matrix44 product;	// Final value
	for(int i=0; i<4; i++){
		for(int j=0; j<4; j++){
			product.element[i][j]= //For each part of the matrix
								A.element[i][0]*B.element[0][j]+
								A.element[i][1]*B.element[1][j]+
								A.element[i][2]*B.element[2][j]+
								A.element[i][3]*B.element[3][j];
		}
	}
	return product;
}
	
Matrix44::Matrix44(void) {
	for (int i=0; i<4; i++){ 
		for (int j=0; j<4; j++) {
			if (i==j) element[i][j] = 1.;
			else      element[i][j] = 0.;
		}
	}
}

Matrix44 Matrix44::transpose(void) const {
	Matrix44 result;
	for (int i=0; i<4; i++){ 
		for (int j=0; j<4; j++){ 
			result.element[i][j] = element[j][i];
		}
	}
	return result;
}

Matrix44 Matrix44::inverse(void) const{ //Inverting matrix
	Matrix44 result;	//Final value
	Matrix44 r;			//Temp value
	for(int i=0; i<3; i++){	//For the internal 3x3
		for(int j=0; j<3; j++){
			result.element[i][j]=element[j][i];	// Transpose the values
		}
	}
	for(int i=0; i<3; i++){
		result.element[i][3]=//For the 3x1 in the upper right
								-result.element[i][0]*element[0][3]+
								-result.element[i][1]*element[1][3]+
								-result.element[i][2]*element[2][3];
							//Multiply the negative of the traspose of the 3x3 and the previous 3x1 in the upper right
	}
	for(int i=0; i<3; i++){
		result.element[3][i]=element[3][i];
		//Copy over the bottom row
	}
	return result;
}

void IKinematics(Matrix44 &A){
	float t1,t2,t3,t4,t5;
	int i,j,k=0;
//	int nx,ny,nz;		|nx sx ax tx| |0  1  2  3 |
//	int sx,sy,sz;		|ny sy ay ty| |4  5  6  7 |
//	int ax,ay,az;		|nz sz az tz| |8  9  10 11|
//	int tx,ty,tz;
	float nsat[12];
	float a,z,q,w;
	Matrix44 A30,A35;

	for(i=0;i<3;i++){
		for(j=0;j<4;j++){
			nsat[k]=A.element[i][j];
			k++;
		}
	}

/*	for(k=0;k<12;k++){
		cout <<endl<< nsat[k]<<endl;
	}*/

	q= (nsat[7]-(10.5*nsat[6]));	//ty-d5*ay
	w= (nsat[3]-(10.5*nsat[2]));	//tx-d5*ax
	
//	t1= atan2((nsat[7]-(10.5*nsat[6])),(nsat[3]-(10.5*nsat[2])))*180/pi;
	t1= atan2(q,w)*180/pi;
	
	a= sqrt(pow(w,2)+pow(q,2));
	z= round(1000*((nsat[11]-(10.5*nsat[10]))-27.2))/1000;
	
	t3= acos(((pow(a,2)+pow(z,2)-pow(19.2,2)-pow(19.2,2))/(2*19.2*19.2)))*180/pi;
	if(isnan(t3)) t3=0;
	
	t2=atan2(z,a)*180/pi+atan2(19.2*sin(t3*pi/180),19.2+19.2*cos(t3*pi/180))*180/pi;

	A30.element[0][0]=cos(t1*pi/180)*cos((t3-t2)*pi/180);
	A30.element[0][1]=sin(t1*pi/180)*cos((t3-t2)*pi/180);
	A30.element[0][2]=-sin((t3-t2)*pi/180);
	A30.element[0][3]=-19.2-19.2*cos(t3*pi/180)+27.2*sin((t3-t2)*pi/180);
	A30.element[1][0]=-cos(t1*pi/180)*sin((t3-t2)*pi/180);
	A30.element[1][1]=-sin(t1*pi/180)*sin((t3-t2)*pi/180);
	A30.element[1][2]=-cos((t3-t2)*pi/180);
	A30.element[1][3]=19.2*sin(t3*pi/180)+27.2*cos((t3-t2)*pi/180);
	A30.element[2][0]=-sin(t1*pi/180);
	A30.element[2][1]=cos(t1*pi/180);
	A30.element[2][2]=0;
	A30.element[2][3]=0;
	A30.element[3][0]=0;
	A30.element[3][1]=0;
	A30.element[3][2]=0;
	A30.element[3][3]=1;
		
	A35=A30*A;
	
	t4=atan2(A35.element[1][2],A35.element[0][2])*180/pi;

	t5=atan2(A35.element[2][0],A35.element[2][1])*180/pi;
	
	t1=round(100*t1)/100;
	t2=round(100*t2)/100;
	t3=round(100*t3)/100;
	t4=round(100*t4)/100;
	t5=round(100*t5)/100;


	cout << endl<< "Theta 1 = "<< t1 << endl;
	cout << "Theta 2 = "<< t2 << endl;
	cout << "Theta 3 = "<< t3 << endl;
	cout << "Theta 4 = "<< t4 << endl;
	cout << "Theta 5 = "<< t5 << endl;


}
Matrix44 FKinematics(float d, float O, float a, float A){
	Matrix44 result;
	result.element[0][0]=cos((O*pi)/180);
	result.element[0][1]=-cos((A*pi)/180)*sin((O*pi)/180);
	result.element[0][2]=sin((A*pi)/180)*sin((O*pi)/180);
	result.element[0][3]=a*cos((O*pi)/180);
	result.element[1][0]=sin((O*pi)/180);
	result.element[1][1]=cos((A*pi)/180)*cos((O*pi)/180);
	result.element[1][2]=-sin((A*pi)/180)*cos((O*pi)/180);
	result.element[1][3]=a*sin((O*pi)/180);
	result.element[2][0]=0;
	result.element[2][1]=sin((A*pi)/180);
	result.element[2][2]=cos((A*pi)/180);
	result.element[2][3]=d;
	result.element[3][0]=0;
	result.element[3][1]=0;
	result.element[3][2]=0;
	result.element[3][3]=1;
	if (A == 90){
		result.element[0][1]=-0;
		result.element[1][1]=0;
		result.element[2][0]=0;
		result.element[2][2]=0;	
	}
	if (O == 90){
		result.element[0][0]=0;
		result.element[0][3]=0;
		result.element[1][1]=0;
		result.element[1][2]=0;
		result.element[2][0]=0;
	}
	if (A == 180){
	result.element[0][2]=0;
	result.element[1][2]=0;	
	result.element[2][1]=0;
	}
	if (O == 180){
	result.element[0][1]=0;
	result.element[0][2]=0;
	result.element[1][0]=0;
	result.element[1][3]=0;
	}
	return result;
}
	
istream& operator>> (istream& s, Matrix44& t) {
	for (int i=0; i<4; i++){ 
		for (int j=0; j<4; j++){ 
			s >> t.element[i][j];
		}
	}
	if (!s) { cerr << "Error reading Matrix from stream";  exit(0); }
return s;
}

ostream& operator<< (ostream& s, const Matrix44& t) {
	for (int i=0; i<4; i++) {
		s <<'|';
		for (int j=0; j<4; j++){ 
				s <<setw(10)<< t.element[i][j]<<setw(10); //<< "   ";
		
		}
		s <<'|'<< endl;		
	}
	if (!s) { cerr << "Error writing Matrix to stream";  exit(0); }
	return s;
}

class Matrix41 {
	private:
		double element[4][1];
	public:
		Matrix41(void);
		Matrix41 transpose(void) const;
		friend istream& operator>> (istream& s, Matrix41& t);
		friend ostream& operator<< (ostream& s, const Matrix41& t);
		friend Matrix41 operator*(const Matrix44 &A, const Matrix41 &P);
};

Matrix41 operator*(const Matrix44 &A, const Matrix41 &P){
	int j=0;
	Matrix41 product;
	for(int i=0; i<4; i++){
			product.element[i][j]=	
						//Keeps the column of the point constant
								A.element[i][0]*P.element[0][0]+
								A.element[i][1]*P.element[1][0]+
								A.element[i][2]*P.element[2][0]+
								A.element[i][3]*P.element[3][0];
	}
	return product;
}	


Matrix41::Matrix41(void) {
	//int j=0;
	//for (int i=0; i<4; i++){ 
			//if (i==j) element[i][j] = 1.;
			//else      element[i][j] = 0.;
		
	//}
	element[0][0]=0;
	element[1][0]=0;
	element[2][0]=0;
	element[3][0]=0;
}

Matrix41 Matrix41::transpose(void) const {
	int j=0;
	Matrix41 result;
	for (int i=0; i<4; i++){ 
			result.element[i][j] = element[j][i];
	}
	return result;
}

istream& operator>> (istream& s, Matrix41& t) {
	int j=0;
	for (int i=0; i<3; i++){ //Only reads 3 values
			s >> t.element[i][j];
	}
	t.element[3][0]=1;	//Adds 4th row with value 1
	if (!s) { cerr << "Error reading Matrix from stream";  exit(0); }
	return s;
}

ostream& operator<< (ostream& s, const Matrix41& t) {
	int j=0;
	for (int i=0; i<3; i++) { // Only prints 3 rows
			s << '|' << setw(3) << t.element[i][j] << setw(3) << '|';
			s << endl;
	}
	if (!s) { cerr << "Error writing Matrix to stream";  exit(0); }
	return s;
}

void clear(void){ // Prints return to clear the terminal
	for(int j=0;j<40;j++){
		cout << endl;
	}
}
int main(int, char* []){                    // Begin the main program
	int i,x,ch;
	float t1,t2,t3,t4,t5;
	float d1=27.2,d2=0,d3=0,d4=0,d5=10.5;
	float A1=90,A2=180,A3=0,A4=90,A5=0;
	float a1=0,a2=19.2,a3=19.2,a4=0,a5=0;
	Matrix44 m,n;
	Matrix44 m1,m2,m3,m4,m5,mf;
	Matrix44 ik;
	Matrix41 p;
	
	while(x){
		cout << endl << "1 - Enter matrix A";
		cout << endl << "2 - Enter matrix B";
		cout << endl << "3 - Enter point P";
		cout << endl << "4 - Multiply";
		cout << endl << "5 - Transform";
		cout << endl << "6 - Inverse";
		cout << endl << "7 - Print matrices";
		cout << endl << "8 - Forward Kinematics";
		cout << endl << "9 - Inverse Kinematics";
		cout << endl << "0 - Quit" << endl;
		
		cin >> i;	//Read input
		switch (i){
			case 1:
				clear();
				cout << endl << "Enter 16 elements of matrix A" << endl;
				cin >> m;	//Assign input to matrix A
				break;
			case 2:
				clear();
				cout << endl << "Enter 16 elements of matrix B" << endl;
				cin >> n;	//Assign input to matrix B
				break;
			case 3:
				clear();
				cout << endl << "Enter 3 elements of point P" << endl;
				cin >> p;	//Assign input to matrix P
				break;
			case 4:
				clear();
				cout<<endl<<"A x B"<< endl<<(m*n)<<endl;
							// Calls operator to multiply matrices	
				break;
			case 5:
				clear();
				cout<<endl<<"A x P"<< endl<<(m*p)<<endl;	
						//Calls operator for 4x4 * 4x1
				break;
			case 6:
				clear();
				cout<<endl<<"Inverse of A"<< endl<<m.inverse()<<endl;
						// Calls inverse
				break;
			case 7:
				clear();
				cout <<"Matrix A"<<endl<< m<<endl;
				cout <<"Matrix B"<<endl<< n<<endl;
				cout <<"Point P"<<endl<< p<<endl;
				break;
			case 8:
				cout << "Input 5 values of theta" << endl;
				cin >> t1 >> t2 >> t3 >> t4 >> t5;
				m1=FKinematics(d1,t1,a1,A1);
				m2=FKinematics(d2,t2,a2,A2);
				m3=FKinematics(d3,t3,a3,A3);
				m4=FKinematics(d4,90+t4,a4,A4);
				m5=FKinematics(d5,t5,a5,A5);
				cout << m1 << endl << m2 << endl << m3 << endl << m4 << endl << m5 << endl;
				mf=m1*m2;
				mf=mf*m3;
				mf=mf*m4;
				mf=mf*m5;
				cout << endl << mf << endl;				
				break;
			case 9:
				cout << "Enter 1 to choose new matrix or 2 to choose matrix from forward kinematics"<< endl;
				cin >> ch;
				if(ch==1){
					cout <<endl;
					cin >> ik;
				}
				if(ch ==2){
					ik=mf;
				}
				IKinematics(ik);

				break;
			case 0: 
				x=0;
				break;
			default:
				cout << endl << "Invalid option" << endl;
				break;
		}
	}
}


