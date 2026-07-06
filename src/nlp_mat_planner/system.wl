(* ::Package:: *)

system[{{u1_,u2_,k_},c_}]:=Module[{k2=k k,su1=Evaluate[Sin[u1]],su2=Evaluate[Sin[u2]],cu1=Evaluate[Cos[u1]],cu2=Evaluate[Cos[u2]],Delta,du1,du2,EE1,EF1,EEF,su12,su22,du12,du22},
	su12=Evaluate[su1 su1];su22=Evaluate[su2 su2];
	Delta=Evaluate[(1-k2 su12 su22)];
	du1=Evaluate[Sqrt[1-k2 su12]];du2=Evaluate[Sqrt[1-k2 su22]];
	du12=Evaluate[du1 du1]; du22=Evaluate[du2 du2];
	EE1=Evaluate[EllipticE[u1,k2]];EF1=Evaluate[EllipticF[u1,k2]];
	Switch[c,
		1,EEF=Evaluate[-2 EE1+EF1];
			{(Delta EEF)/(2 du1 du2 k su1 su2)+(cu1 k su2)/du2,   (cu2 Delta (cu1 EEF+du1 su1))/(4 du12 du22 k su12 su22),
			 1/(48 du12 du1 du22 du2 k2 k su12 su1 su22 su2) (-Delta^3 (EE1+EEF-EEF k2)+cu1 du1 k2 su1 (4-3 (1+2 k2 su12) su22-6 k2 su12 (-2+k2 su12) su22^2+k2^2 su12 (-8+(-1+8 k2) su12) su22^3)+3 Delta k2 (cu1^2 cu2^2 EEF+2 cu1 cu2 du1 du2 EEF su1 su2+du12 du2 su12 su2 (2 cu2+du2 EEF su2)))},
		{2,_},EEF=Evaluate[ 2EE1+EF1 (-2+k2)];
			{(-((Delta EEF)/(cu1 k2 su1))+2 du1 su22)/(2 cu2 c[[2]] su2),  (Delta du2 (-du1 EEF+cu1 k2 su1))/(4  (1-su12)(1-su22) k2^2 c[[2]] su12 su22),
			 1/(96 cu1^3 cu2^3 k2^3 c[[2]] su12 su1 su22 su2) (Delta^3 (-EF1 k2^2+EEF (-2+k2))+2 cu1 du1 k2 su1 (4-3 k2 (1+2 su12) su22-k2^3 su12^2 su22^3+2 k2^2 su12 su22^2 (6-3 su12+4 (-1+su12) su22))+6 Delta (2 cu1^2 cu2 du2 k2^2 su12 su2+EEF (-1+k2 (su12-2 cu1 cu2 du1 du2 su1 su2+su22)+k2^2 su12 su22 (-2+su12+su22-su12 su22))))}
	]
];
Jac[{{u1_,u2_,k_},c_}]:=Module[{k2=k k,k4,k6,su1=Evaluate[Sin[u1]],su2=Evaluate[Sin[u2]],cu1=Evaluate[Cos[u1]],cu2=Evaluate[Cos[u2]],Delta,du1,du2,EE1,EF1,EEF,su12,su22,su14,su24,du12,du22},
	k4=Evaluate[k2 k2]; k6=Evaluate[k2 k4];
	su12=Evaluate[su1 su1];su22=Evaluate[su2 su2]; su14=Evaluate[su12 su12];su24=Evaluate[su22 su22];
	Delta=Evaluate[(1-k2 su12 su22)];
	du1=Evaluate[Sqrt[1-k2 su12]];du2=Evaluate[Sqrt[1-k2 su22]];
	du12=Evaluate[du1 du1]; du22=Evaluate[du2 du2];
	EE1=Evaluate[EllipticE[u1,k2]];EF1=Evaluate[EllipticF[u1,k2]];
	Switch[c,
		1,EEF=Evaluate[-2 EE1+EF1];
			{{-(((cu1 EEF+du1 su1) (1+k2 su12 (-2+su22)))/(2 du1 du12 du2 k su12 su2)),
			  (cu1 cu2 k)/(du2 du22)-(cu2 EEF (1+k2 (-2+su12) su22))/(2 du1 du2 du22 k su1 su22),
			  ((du1 (-Delta du1 du22 EE1+cu1 k2 su1 (1+(-2+k2 (1+su12)+k4 su12 (-2+su22)) su22)))/((-1+k2) k2)+EEF (su22+su12 (1+su22 (-2+k2 (-2+su12+su22)))))/(2 du1 du12 du2 du22 su1 su2)},
			 {(cu2 (-2 cu1 du1 su1 (1-2 k2 su12+k4 su14 su22)+EEF (-2+su12 (1+k2 (4+su12 (-3+(1+k2 (-2+su12)) su22))))))/(4 du12^2 du22 k su1 su12 su22),
			  ((cu1 EEF+du1 su1) (-2+su22 (1+k2 (4+(-3+su12 (1+k2 (-2+su22))) su22))))/(4 du12 du22^2 k su12 su2 su22),
			  (cu2 (2 cu1 EEF (su22+su12 (1-(1+2 k2) su22+k4 su12 su24))+(du1 (-cu1 Delta du1 du22 EE1+su1 (1+k2 ((-3+2 k2) su22+su12 (-3+su22+k2 (2+su22 (3-4 k2+su22)))+su14 (k2 su22+(-3+2 k2) k4 su24)))))/((-1+k2) k2)))/(4 du12^2 du22^2 su12 su22)},
			{1/(16 du1 du12^2 du2 du22 k k2 su14 su2 su22) (du1 k2 su1 (-4-4 cu2 du2 EEF su2+su12 (2+2 cu2 du2 EEF su2+k2 (8+8 cu2 du2 EEF su2-4 su22)-3 su22)+3 su22+k4 su12 su14 su22 (2+2 cu2 du2 EEF su2+su22 (-4-2 k2 (-2+su22)+su22))+k2 su14 (-6+su2 (2 (5-4 k2) su2+2 cu2 du2 EEF (-3+(1-2 k2) su22)+su2 su22 (-2+k2 (2+su22)))))+cu1 (Delta^2 EE1 (1+k2 su12 (-2+su22))-4 cu2 du12 du2 k2 su12 su2 (1-2 k2 su12+k4 su14 su22)+EEF (1+k2 (-4+3 su22+su12 (-1-3 su22+k2 (8-4 su22+su12 (-4+su22 (-3 (-4+su22)+k2 (-8+su22 (2+su22+su12 (-3-2 k2 (-2+su22)+su22))))))))))),
			 1/(16 du1 du12 du2 du22^2 k k2 su1 su12 su24) (2 du2 k2 su1 (cu1 du1 EEF+su1-k2 su1 su12) su2 (-2+su22 (1+k2 (4+(-3+su12 (1+k2 (-2+su22))) su22)))+cu2 (-4 cu1 du1 k2 su1+EEF (1+k2 (-4+3 su12))+Delta^2 EE1 (1+k2 (-2+su12) su22)+k2 su22 (EEF (-1-3 su12+k2 (8-4 su22+su12 (-4+su22 (-3 (-4+su12)+k2 (-8+su12 (2+su12+(-3-2 k2 (-2+su12)+su12) su22))))))+cu1 du1 su1 (1+k2 (2 (4+su12)-2 (2+su12 (-2+k2 (4+su12))) su22+k2 (-1+4 k2) su14 su24))))),
			 1/(16 du1 du12^2 du2 du22^2 k4 su1 su12 su2 su22) (EEF (1-k2 (2 su22+su12 (2+su22))-k4 k6 (-2+su12) su12 su14 (-2+su22) su22 su24-k4^2 su14 su24 (-8+su14-4 cu1 cu2 du1 du2 su1 su2+su12 (2-3 su22)+2 su22+su24)+k6 su12 su22 (-8 cu1 cu2 du1 du2 su1 su2+su12 (4-11 su22)+4 (-2+su22)+su14 su24)+k4 (-4 cu1 cu2 du1 du2 su1 su12 su2 (-1+su22)+4 su22+4 cu1 cu2 du1 du2 su1 su2 su22+su14 (-3+10 su22-5 su24)-3 su24+su12 (4-7 su22+10 su24)))+1/(-1+k2) (-Delta EE1 (1+k2 (2 cu1 cu2 du1 du2 su1 su2-3 su22+su12 (-3+2 su22))+k4^2 (-2+su12) su14 (-2+su22) su24+k6 su12 su22 (-2+su14+2 cu1 cu2 du1 du2 su1 su2-3 su22+su12 (-3+2 su22)+su24)+k4 (-2 cu1 cu2 du1 du2 su1 su12 su2+su22-2 cu1 cu2 du1 du2 su1 su2 su22+su12 (1+5 su22-su24)+su24-su14 (-1+su22+su24)))+du1 k2 su1 (cu1 (1-su22+k2 (su22 (-5+4 su22)+su12 (-5+8 su22-4 su24))-k6 su12 su22 (8-4 su22+su14 su22+su12 (2+8 su22-3 su24))+2 k4 k6 su12 su14 (-2+su22) su22 su24-k4^2 su14 su24 (-8+2 su22+su24+su12 (-2-2 su22+su24))+k4 (4 su22+su12 (4+2 su22-su24)-3 su24+su14 su22 (2-su22+su24)))+2 cu2 du1 du2 su1 su2 (1+k2 (su12 (-3+su22)-3 su22)-k6 su12 su22 (4+3 su12 su22)+2 k4^2 su14 su24+k4 (2 su22+su14 su22+su12 (2+3 su22+su24))))))}},
		{2,_},EEF=Evaluate[2EE1+EF1 (-2+k2)];
			{{((-du1 EEF+cu1 k2 su1) (-1+(1+du22) su12))/(2 c[[2]] cu2 du1 k2 (1-su12) su12 su2),
			  (2 du1+(EEF (1-(1+du12) su22))/(cu1 k2 su1 su22))/(2 c[[2]] (1-su22)),
			  -((du1 (EEF (4+(-4+Delta) k2)+Delta EF1 k4)-2 cu1 k2 (2+(-2+Delta) k2) su1)/(4 c[[2]] cu1 cu2 du1 (-1+k) k (1+k) k2 su1 su2))},
			 {(du2 (2 du1 k2 su1 (-1+su12) (-1+2 su12+(-1+du22) su14)+cu1 EEF (-2+su12 (4+k2 (1+su12 (-3+(-2+k2 (1+su12)) su22))))))/(4 c[[2]] du1 k4 su1 (-1+su12)^2 su12 (-1+su22) su22),
 			 (cu2 (du1 EEF-cu1 k2 su1) (-2+su22 (4+k2 (1+su22 (-3+su12 (-2+k2 (1+su22)))))))/(4 c[[2]] du2 k4 (-1+su12) su12 su2 (-1+su22)^2 su22),
			  (du1 k2 (8 cu1 su1+k2 (du1 du22 EF1 (-1+su22)-du1 du12 du22 EF1 su22+2 cu1 su1 (-3+su22 (-3-2 su12+k2 (2+su12 (1+su22))))))+EEF (-8+k2 (7+(6-5 k2) su22-(-1+du22) su14 (-2+k2+k4 su22)+su12 (6+4 su22+k2 (-5+(-7+3 k2) su22+(-2+k2) su24)))))/(8 c[[2]] du1 du2 k (-1+k2) k4 (-1+su12) su12 (-1+su22) su22)},
			 {(2 cu1 k2 su1 (-1-3 du22+2 cu2 du2 du22 EEF k2 su14 su2+su12 (8+k2 (2-4 su22)-3 k4 su22)+2 cu2 du2 EEF su2 (2-(2+2 Delta+k2) su12+2 Delta k2 su14+k4 su12 su14 su22)+k4 su12 su14 su22 (2-4 (-1+k2) su22+(-2+k2) k2 su24)+k2 su14 (-6+2 (-4+5 k2) su22-2 (-1+k2) k2 su24+k4 su22 su24))+du1 (k4 (Delta^3 EF1 (-1+su12)+Delta^2 EF1 (2-3 su12-(-1+du22) su14)-8 cu2 du2 (-1+su12) su12 (-1+2 su12+(-1+du22) su14) su2)+EEF (8-16 su12+k2 (4+4 su12 (-5+su22)-6 su22+8 su14 (1+su22)+Delta (-5+8 su14 su22+4 su12 (5+su22)))-k4^2 su12 su14 su22 su24+k6 su14 (5-2 su22+4 su12 (1+su22)) su24)))/(32 c[[2]] cu2 du1 k6 (1-su12)^2 su12^2 su2 (1-su22) su22),
 			 (-4 cu2 k2 su1 (cu1 du1 EEF+k2 su1 (-1+su12)) su2 (-2+(4+k2) su22+k2 (-3+(-2+k2) su12) su24+k4 su12 su22 su24)+du2 (-k2 (Delta^2 EF1 k2 (-1+(1+du12) su22)+2 cu1 du1 su1 (4-(8+k2 (1+2 su12)) su22+2 k2 (2-2 (-2+k2) su12+k2 su14) su24+(-1+du12)^2 (-4+k2) su22 su24))+EEF (8-16 su22-k4^2 su12 su14 su22 su24+k6 su14 (5+4 su22+su12 (-2+4 su22)) su24+k2 (4-20 su22+8 su24+su12 (-6+4 su22+8 su24)+Delta (-5+4 (5+su12) su22+8 su12 su24)))))/(32 c[[2]] cu1 du2 k6 su1 (1-su12) su12 (1-su22)^2 su22^2),
 			 (2 cu1 cu2 du12 du22 k2 su1 su2 (-Delta EF1 k4+EEF (-12+k2 (4+7 Delta+8 su12 su22)))+2 cu1 k2 su1 (2 cu2 EEF (-1+k2) su2 (-2+k2 (-4+2 su12 su22+Delta (4+3 su12+3 su22))+4 k6 su14 su24)+du2 (8-k6 su12 su22 (3+su12 (2+3 su22))-k2 (7+4 su22+su12 (6+8 su22))+k4 k6 su12 su14 su22 su24+k4^2 su14 (-2+su12 (1-2 su22)+su22) su24+k4 (3 su22+su12 (5+12 su22)+2 su14 (su22+2 su24))))+du1 (-4 cu2 k4 (-1+su12) su12 su2 (4-k2 (3+(3+2 su12) su22)+k4 su22 (2+su12 (1+su22)))+du2 (EF1 k4 (-2+k2 (-2+3 Delta+su12+su22+2 su12 su22)+k4^2 su12 su14 su22 su24-k6 su14 (-1+su12+su22) su24)-EEF (16-k6 su12 su22 (7+6 su12 su22)-8 k2 (2+su22+su12 (1+2 su22))+k4 k6 su12 su14 su22 su24+k4^2 su14 (-3+su12 (1-2 su22)+su22) su24+k4 (1+7 su22+su12 (7+24 su22)+8 su14 su24)))))/(32 c[[2]] cu1 cu2 du1 du2 k (-1+k2) k6 su1 (1-su12) su12 su2 (1-su22) su22)}}
	]
];
x[{{u1_,u2_,k_},c_}]:=Module[{su1=Sin[u1],su2=Sin[u2],k2=k k,su12,su22},su12=su1 su1;su22=su2 su2;
	Switch[c,
		1,(4 k su1 Sqrt[1-k2 su12] su2 Sqrt[1-k2 su22])/(-1+k2 su12 su22),
		{2,_},(k c[[2]] Sin[2 u1] Sin[2 u2])/(-1+k2 su12 su22)
	]
];
EE=Function[{phi,k2}, EllipticE[JacobiAmplitude[phi,k2],k2]];
sn=Function[{phi,k2}, JacobiSN[phi,k2]];
cn=Function[{phi,k2}, JacobiCN[phi,k2]];
dn=Function[{phi,k2}, JacobiDN[phi,k2]];
xt[{{phi_,k_,alpha_},c_}] := Module[{psi,sigma = Sqrt[Abs[alpha]],k2=k k},
	Switch[c,
	1,
		Function[t, 2 k sigma (cn[sigma (phi+t),k2] - cn[sigma phi,k2]) / alpha],
	{2,_}, psi = phi/k;
		Function[t, 2 sigma c[[2]] (dn[sigma (psi+t/k),k2] - dn[sigma psi,k2]) / (alpha k)]
	]
];
yt[{{phi_,k_,alpha_},c_}] := Module[{psi,sigma = Sqrt[Abs[alpha]],k2=k k},
	Switch[c,
	1,
		Function[t, 2 sigma (EE[sigma (phi+t),k2] - EE[sigma phi,k2]) / alpha - Sign[alpha] t],
	{2,_}, psi = phi/k;
		Function[t, (k2-2) Sign[alpha] t / k2 + 2 sigma (EE[sigma (psi+t/k),k2] - EE[sigma psi, k2]) / (alpha k)]
	]
];
zt[{{phi_,k_,alpha_},c_}] := Module[{psi, sigma = Sqrt[Abs[alpha]],k2=k k},
	Switch[c,
		1,
			Function[t, 2 k (sn[sigma (phi+t),k2] dn[sigma (phi+t),k2] - sn[sigma phi,k2] dn[sigma phi,k2] - alpha yt[{{phi,k,alpha},c}][t] (cn[sigma (phi+t),k2]+cn[sigma phi,k2])/ (2 sigma))/ Abs[alpha]],
		{2,_}, psi = phi/k;
			Function[t, - yt[{{phi,k,alpha},c}][t] (xt[{{phi,k,alpha},c}][t]/2 + 2 sigma c[[2]] dn[sigma psi,k2] / (alpha k)) + 2 c[[2]] (sn[sigma (psi + t/k),k2] cn[sigma (psi + t/k),k2] - sn[sigma psi,k2] cn[sigma psi,k2])/ Abs[alpha]]
	]
];
vt[{{phi_,k_,alpha_},c_}]:=Module[{psi, sigma = Sqrt[Abs[alpha]],k2=k k},
	Switch[c,
		1,
			Function[t,((yt[{{phi,k,alpha},c}][t])^3)/6+2*k2*cn[sigma phi,k2]^2*yt[{{phi,k,alpha},c}][t]/sigma^2-4*k^2*cn[sigma phi,k2]*(sn[sigma (phi+t),k^2]*dn[sigma (phi+t),k2]-dn[sigma phi,k2]*sn[sigma phi,k2])/sigma^3+2 k2*(2/3 sn[sigma (phi+t),k2]*cn[sigma (phi+t),k2]*dn[sigma (phi+t),k2]-2/3 sn[sigma phi,k2]*cn[sigma phi,k2]*dn[sigma phi,k2] + (1-k2) sigma t /(3*k2)+(2*k2-1)*(EE[sigma (phi+t),k2] - EE[sigma phi,k2])/(3 k2) )/sigma^3],
		{2,_}, psi = phi/k;
			Function[t, (yt[{{phi,k,alpha},c}][t]^3)/6+2/k2 dn[sigma psi,k2]^2*yt[{{phi,k,alpha},c}][t]/sigma^2-4/(k sigma^3)*dn[sigma psi,k2]*(sn[sigma (psi+t/k),k2]*cn[sigma (psi+t/k),k2]-cn[sigma psi,k2]*sn[sigma psi,k2])+4/(k sigma^3)*(1/3 sn[sigma (psi+t/k),k2]*cn[sigma (psi+t/k),k2]*dn[sigma (psi+t/k),k2]-1/3 sn[sigma psi,k2]*cn[sigma psi,k2]*dn[sigma psi,k2] - (1-k2)sigma t/(3*k2 k)-(k2-2)*(EE[sigma (psi+t/k),k2] - EE[sigma psi,k2])/(6 k2) )]
	]
];
xyzv[{{phi_,k_,alpha_},c_}]:=
Module[{sigma = Sqrt[Abs[alpha]],
		k2=k k,
		ct,c0,st,s0,dt,d0,EEt0, 
		yt,xt,psi,argt,arg0},
	Switch[c,
	1,  
		Function[t, 
			argt=Evaluate[sigma (phi+t)]; arg0=Evaluate[sigma phi];
			ct=Evaluate[cn[argt,k2]]; c0=Evaluate[cn[arg0,k2]];
			st=Evaluate[sn[argt,k2]]; s0=Evaluate[sn[arg0,k2]];
			dt=Evaluate[dn[argt,k2]]; d0=Evaluate[dn[arg0,k2]];
			EEt0 = Evaluate[EE[argt,k2] - EE[arg0,k2]];
			yt=Evaluate[2 sigma EEt0 / alpha - Sign[alpha] t];		
			{2 k sigma (ct - c0) / alpha,
			 yt,
			 2 k (st dt - s0 d0 - alpha yt (ct+c0)/ (2 sigma))/ Abs[alpha],
			 yt^3/6+2*k2*c0^2*yt/sigma^2-4*k2*c0*(st*dt-d0*s0)/sigma^3+2 k2*(2/3 st*ct*dt-2/3 s0*c0*d0 + (1-k2) sigma t /(3*k2)+(2*k2-1) EEt0/(3 k2))/sigma^3
			}
		],
	{2,_}, psi = phi/k;
		Function[t, 
			argt=Evaluate[sigma (psi+t/k)]; arg0=Evaluate[sigma psi];
			ct=Evaluate[cn[argt,k2]]; c0=Evaluate[cn[arg0,k2]];
			st=Evaluate[sn[argt,k2]]; s0=Evaluate[sn[arg0,k2]];
			dt=Evaluate[dn[argt,k2]]; d0=Evaluate[dn[arg0,k2]];
			EEt0 = Evaluate[EE[argt,k2] - EE[arg0,k2]];
			xt=Evaluate[2 sigma c[[2]] (dt - d0) / (alpha k)];
			yt= Evaluate[(k2-2) Sign[alpha] t / k2 + 2 sigma EEt0 / (alpha k)];
			{xt,
			 yt,
			 - yt (xt/2 + 2 sigma c[[2]] d0 / (alpha k)) + 2 c[[2]] (st ct - s0  c0)/ Abs[alpha],
			 yt^3/6+2/k2 d0^2*yt/sigma^2-4/(k sigma^3)*d0*(st*ct-c0*s0)+4/(k sigma^3)*(1/3 st*ct*dt-1/3 s0*c0*d0 - (1-k2)sigma t/(3*k2 k)-(k2-2) EEt0/(6 k2) )
			 }
		]
	]
];

fz[u1_,k_]:=Module[{k2=k k},Cos[u1] (-2 EllipticE[u1,k2]+EllipticF[u1,k2])+Sin[u1] Sqrt[1-k2 Sin[u1]^2]];

YZ3[{p_,tau_}]:=Module[{sp=Evaluate[Sinh[p]],cp=Evaluate[Cosh[p]],stau=Evaluate[Sinh[tau]],ctau=Evaluate[Cosh[tau]],sp2,stau2},
	sp2=Evaluate[sp sp]; stau2=Evaluate[stau stau];
	{(-2 cp sp+p (1+sp2+stau2))/(2 sp stau),(ctau (cp p-sp) (1+sp2+stau2))/(4 sp2 stau2)}
]
W3[{p_,tau_}]:=Module[{sp=Evaluate[Sinh[p]],cp=Evaluate[Cosh[p]],stau=Evaluate[Sinh[tau]],ctau=Evaluate[Cosh[tau]],sp2,stau2},
	sp2=Evaluate[sp sp]; stau2=Evaluate[stau stau];
	(3 p (1+sp2+stau2) (1+2 cp ctau sp stau+stau2+sp2 (1+2 stau2))-sp (6 ctau sp stau (1+sp2+stau2)+cp (sp2^2+3 (1+stau2)^2+4 sp2 (1+3 stau2))))/(48 sp2 sp stau2 stau)
]
YW3[{p_,tau_}]:=Module[{sp=Evaluate[Sinh[p]],cp=Evaluate[Cosh[p]],stau=Evaluate[Sinh[tau]],ctau=Evaluate[Cosh[tau]],sp2,stau2},
	sp2=Evaluate[sp sp]; stau2=Evaluate[stau stau];
	{(-2 cp sp+p (1+sp2+stau2))/(2 sp stau),
 	(3 p (1+sp2+stau2) (1+2 cp ctau sp stau+stau2+sp2 (1+2 stau2))-sp (6 ctau sp stau (1+sp2+stau2)+cp (sp2^2+3 (1+stau2)^2+4 sp2 (1+3 stau2))))/(48 sp2 sp stau2 stau)}
];
Z3[{p_,tau_}]:=Module[{sp=Evaluate[Sinh[p]],cp=Evaluate[Cosh[p]],stau=Evaluate[Sinh[tau]],ctau=Evaluate[Cosh[tau]],sp2,stau2},
	sp2=Evaluate[sp sp]; stau2=Evaluate[stau stau];
	(ctau (cp p-sp) (1+sp2+stau2))/(4 sp2 stau2)
];


system3[{p_,tau_}]:=Module[{sp=Evaluate[Sinh[p]],cp=Evaluate[Cosh[p]],stau=Evaluate[Sinh[tau]],ctau=Evaluate[Cosh[tau]],sp2,stau2},
	sp2=Evaluate[sp sp]; stau2=Evaluate[stau stau];
	{(-2 cp sp+p (1+sp2+stau2))/(2 sp stau),(ctau (cp p-sp) (1+sp2+stau2))/(4 sp2 stau2),
 	(3 p (1+sp2+stau2) (1+2 cp ctau sp stau+stau2+sp2 (1+2 stau2))-sp (6 ctau sp stau (1+sp2+stau2)+cp (sp2^2+3 (1+stau2)^2+4 sp2 (1+3 stau2))))/(48 sp2 sp stau2 stau)}
];
(*wt[{{phi_,k_,alpha_},c_}]:=Module[{psi, sigma = Sqrt[Abs[alpha]]},
	Switch[c,
		1,Function[t, 2 k^2 cn[sigma phi,k]^2 Evaluate[yt[{{phi,k,alpha},c}][t]] / Abs[alpha]  + 4 k^2 (cn[sigma (phi + t),k]sn[sigma (phi + t),k]dn[sigma (phi + t),k]/3 -cn[sigma phi,k]sn[sigma phi,k]dn[sigma phi,k]/3 + (1-k^2) sigma t/(6 k^3) + (2 k^2 - 1) (EE[sigma (phi+t),k]-EE[sigma phi,k])/(6 k^2) - cn[sigma phi,k] (sn[sigma (phi + t),k] dn[sigma (phi + t),k] - sn[sigma phi,k] dn[sigma phi,k])) / (sigma alpha)],
	{2,_}, psi = phi/k;
	Function[t,2/k^2 JacobiDN[Abs[sigma] psi,k^2]^2*yt[{{phi,k,alpha},c}][t]/sigma^2-4/(k sigma^3)*JacobiDN[Abs[sigma] psi,k^2]*(JacobiSN[Abs[sigma] (psi+t/k),k^2]*JacobiCN[Abs[sigma] (psi+t/k),k^2]-JacobiCN[Abs[sigma] psi,k^2]*JacobiSN[Abs[sigma] psi,k^2])+4/(k sigma^3)*(1/3 JacobiSN[Abs[sigma] (psi+t/k),k^2]*JacobiCN[Abs[sigma] (psi+t/k),k^2]*JacobiDN[Abs[sigma] (psi+t/k),k^2]-1/3 JacobiSN[Abs[sigma] psi,k^2]*JacobiCN[Abs[sigma] psi,k^2]*JacobiDN[Abs[sigma] psi,k^2] - (1-k^2)Abs[sigma] t/(3*k^3)-(k^2-2)*(EllipticE[ JacobiAmplitude[Abs[sigma] (psi+t/k),k^2],k^2] - EllipticE[ JacobiAmplitude[Abs[sigma] psi,k^2],k^2])/(6 k^2) )]
	]
];*)

