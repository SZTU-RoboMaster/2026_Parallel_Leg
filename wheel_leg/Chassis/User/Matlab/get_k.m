%计算不同腿长下适合的K矩阵，再进行多项式拟合，得到2*6矩阵每个参数对应的多项式参数
tic
j=1;
leg=0.05:0.01:0.45; %100mm ~步长10mm ~ 400mm
for i=leg
    k=get_k_length(i);  % lqr函数返回一个 p x n 的矩阵
    k11(j) = k(1,1);    % T: theta的增益
    k12(j) = k(1,2);    % T: theta_dot的增益
    k13(j) = k(1,3);    % T: x的增益
    k14(j) = k(1,4);    % T: x_dot的增益
    k15(j) = k(1,5);    % T: phi的增益
    k16(j) = k(1,6);    % T: phi_dot的增益

    k21(j) = k(2,1);
    k22(j) = k(2,2);
    k23(j) = k(2,3);
    k24(j) = k(2,4);
    k25(j) = k(2,5);
    k26(j) = k(2,6);
    j=j+1;
end

a11=polyfit(leg,k11,3); % a11(1)*x^3 + a11(2)x^2 + a11(3)x^1 + a11(4) a11返回4个值
a12=polyfit(leg,k12,3);
a13=polyfit(leg,k13,3);
a14=polyfit(leg,k14,3);
a15=polyfit(leg,k15,3);
a16=polyfit(leg,k16,3);

a21=polyfit(leg,k21,3);
a22=polyfit(leg,k22,3);
a23=polyfit(leg,k23,3);
a24=polyfit(leg,k24,3);
a25=polyfit(leg,k25,3);
a26=polyfit(leg,k26,3);


fprintf('float wheel_fitting_factor[6][4] = {\n');
fprintf('{%ff,%ff,%ff,%ff},\n',a11(1,1),a11(1,2),a11(1,3),a11(1,4));
fprintf('{%ff,%ff,%ff,%ff},\n\n',a12(1,1),a12(1,2),a12(1,3),a12(1,4));
fprintf('{%ff,%ff,%ff,%ff},\n',a13(1,1),a13(1,2),a13(1,3),a13(1,4));
fprintf('{%ff,%ff,%ff,%ff},\n\n',a14(1,1),a14(1,2),a14(1,3),a14(1,4));
fprintf('{%ff,%ff,%ff,%ff},\n',a15(1,1),a15(1,2),a15(1,3),a15(1,4));
fprintf('{%ff,%ff,%ff,%ff}\n',a16(1,1),a16(1,2),a16(1,3),a16(1,4));
fprintf('};');

fprintf('float joint_fitting_factor[6][4] = {\n');
fprintf('{%ff,%ff,%ff,%ff},\n',a21(1,1),a21(1,2),a21(1,3),a21(1,4));
fprintf('{%ff,%ff,%ff,%ff},\n\n',a22(1,1),a22(1,2),a22(1,3),a22(1,4));
fprintf('{%ff,%ff,%ff,%ff},\n',a23(1,1),a23(1,2),a23(1,3),a23(1,4));
fprintf('{%ff,%ff,%ff,%ff},\n\n',a24(1,1),a24(1,2),a24(1,3),a24(1,4));
fprintf('{%ff,%ff,%ff,%ff},\n',a25(1,1),a25(1,2),a25(1,3),a25(1,4));
fprintf('{%ff,%ff,%ff,%ff}\n',a26(1,1),a26(1,2),a26(1,3),a26(1,4));
fprintf('};');

















% %计算不同腿长下适合的K矩阵，再进行多项式拟合，得到4*10矩阵每个参数对应的多项式参数
% tic
% j=1;
% leg=0.1:0.01:0.4; %100mm ~步长10mm ~ 400mm
% for i=leg
%     k=get_k_length(i);  % lqr函数返回一个 p x n 的矩阵
%  
%     % 存储K矩阵的各个元素
%     % Tl对应的行
%     k11(j) = k(1,1);    % Tl: x的增益
%     k12(j) = k(1,2);    % Tl: x_dot的增益  
%     k13(j) = k(1,3);    % Tl: yaw的增益
%     k14(j) = k(1,4);    % Tl: yaw_dot的增益
%     k15(j) = k(1,5);    % Tl: theta_l的增益
%     k16(j) = k(1,6);    % Tl: theta_l_dot的增益
%     k17(j) = k(1,7);    % Tl: theta_r的增益
%     k18(j) = k(1,8);    % Tl: theta_r_dot的增益
%     k19(j) = k(1,9);    % Tl: pit的增益
%     k110(j) = k(1,10);  % Tl: pit_dot的增益
%     
%     % Tr对应的行
%     k21(j) = k(2,1);    % Tr: x的增益
%     k22(j) = k(2,2);    % Tr: x_dot的增益
%     k23(j) = k(2,3);    % Tr: yaw的增益
%     k24(j) = k(2,4);    % Tr: yaw_dot的增益
%     k25(j) = k(2,5);    % Tr: theta_l的增益
%     k26(j) = k(2,6);    % Tr: theta_l_dot的增益
%     k27(j) = k(2,7);    % Tr: theta_r的增益
%     k28(j) = k(2,8);    % Tr: theta_r_dot的增益
%     k29(j) = k(2,9);    % Tr: pit的增益
%     k210(j) = k(2,10);  % Tr: pit_dot的增益
%     
%     % Tpl对应的行
%     k31(j) = k(3,1);    % Tpl: x的增益
%     k32(j) = k(3,2);    % Tpl: x_dot的增益
%     k33(j) = k(3,3);    % Tpl: yaw的增益
%     k34(j) = k(3,4);    % Tpl: yaw_dot的增益
%     k35(j) = k(3,5);    % Tpl: theta_l的增益
%     k36(j) = k(3,6);    % Tpl: theta_l_dot的增益
%     k37(j) = k(3,7);    % Tpl: theta_r的增益
%     k38(j) = k(3,8);    % Tpl: theta_r_dot的增益
%     k39(j) = k(3,9);    % Tpl: pit的增益
%     k310(j) = k(3,10);  % Tpl: pit_dot的增益
%     
%     % Tpr对应的行
%     k41(j) = k(4,1);    % Tpr: x的增益
%     k42(j) = k(4,2);    % Tpr: x_dot的增益
%     k43(j) = k(4,3);    % Tpr: yaw的增益
%     k44(j) = k(4,4);    % Tpr: yaw_dot的增益
%     k45(j) = k(4,5);    % Tpr: theta_l的增益
%     k46(j) = k(4,6);    % Tpr: theta_l_dot的增益
%     k47(j) = k(4,7);    % Tpr: theta_r的增益
%     k48(j) = k(4,8);    % Tpr: theta_r_dot的增益
%     k49(j) = k(4,9);    % Tpr: pit的增益
%     k410(j) = k(4,10);  % Tpr: pit_dot的增益
%  
%     j=j+1;
% end

 
% % Tl对应的10个参数的拟合
% a11=polyfit(leg,k11,3);
% a12=polyfit(leg,k12,3);
% a13=polyfit(leg,k13,3);
% a14=polyfit(leg,k14,3);
% a15=polyfit(leg,k15,3);
% a16=polyfit(leg,k16,3);
% a17=polyfit(leg,k17,3);
% a18=polyfit(leg,k18,3);
% a19=polyfit(leg,k19,3);
% a110=polyfit(leg,k110,3);
% 
% % Tr对应的10个参数的拟合
% a21=polyfit(leg,k21,3);
% a22=polyfit(leg,k22,3);
% a23=polyfit(leg,k23,3);
% a24=polyfit(leg,k24,3);
% a25=polyfit(leg,k25,3);
% a26=polyfit(leg,k26,3);
% a27=polyfit(leg,k27,3);
% a28=polyfit(leg,k28,3);
% a29=polyfit(leg,k29,3);
% a210=polyfit(leg,k210,3);
% 
% % Tpl对应的10个参数的拟合
% a31=polyfit(leg,k31,3);
% a32=polyfit(leg,k32,3);
% a33=polyfit(leg,k33,3);
% a34=polyfit(leg,k34,3);
% a35=polyfit(leg,k35,3);
% a36=polyfit(leg,k36,3);
% a37=polyfit(leg,k37,3);
% a38=polyfit(leg,k38,3);
% a39=polyfit(leg,k39,3);
% a310=polyfit(leg,k310,3);
% 
% % Tpr对应的10个参数的拟合
% a41=polyfit(leg,k41,3);
% a42=polyfit(leg,k42,3);
% a43=polyfit(leg,k43,3);
% a44=polyfit(leg,k44,3);
% a45=polyfit(leg,k45,3);
% a46=polyfit(leg,k46,3);
% a47=polyfit(leg,k47,3);
% a48=polyfit(leg,k48,3);
% a49=polyfit(leg,k49,3);
% a410=polyfit(leg,k410,3);
% 
% % 输出Tl的拟合系数
% fprintf('float Tl_fitting_factor[10][4] = {\n');
% fprintf('{%ff,%ff,%ff,%ff},\n', a11(1), a11(2), a11(3), a11(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a12(1), a12(2), a12(3), a12(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a13(1), a13(2), a13(3), a13(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a14(1), a14(2), a14(3), a14(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a15(1), a15(2), a15(3), a15(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a16(1), a16(2), a16(3), a16(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a17(1), a17(2), a17(3), a17(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a18(1), a18(2), a18(3), a18(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a19(1), a19(2), a19(3), a19(4));
% fprintf('{%ff,%ff,%ff,%ff}\n', a110(1), a110(2), a110(3), a110(4));
% fprintf('};\n\n');
% 
% % 输出Tr的拟合系数
% fprintf('float Tr_fitting_factor[10][4] = {\n');
% fprintf('{%ff,%ff,%ff,%ff},\n', a21(1), a21(2), a21(3), a21(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a22(1), a22(2), a22(3), a22(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a23(1), a23(2), a23(3), a23(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a24(1), a24(2), a24(3), a24(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a25(1), a25(2), a25(3), a25(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a26(1), a26(2), a26(3), a26(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a27(1), a27(2), a27(3), a27(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a28(1), a28(2), a28(3), a28(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a29(1), a29(2), a29(3), a29(4));
% fprintf('{%ff,%ff,%ff,%ff}\n', a210(1), a210(2), a210(3), a210(4));
% fprintf('};\n\n');
% 
% % 输出Tpl的拟合系数
% fprintf('float Tpl_fitting_factor[10][4] = {\n');
% fprintf('{%ff,%ff,%ff,%ff},\n', a31(1), a31(2), a31(3), a31(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a32(1), a32(2), a32(3), a32(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a33(1), a33(2), a33(3), a33(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a34(1), a34(2), a34(3), a34(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a35(1), a35(2), a35(3), a35(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a36(1), a36(2), a36(3), a36(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a37(1), a37(2), a37(3), a37(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a38(1), a38(2), a38(3), a38(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a39(1), a39(2), a39(3), a39(4));
% fprintf('{%ff,%ff,%ff,%ff}\n', a310(1), a310(2), a310(3), a310(4));
% fprintf('};\n\n');
% 
% % 输出Tpr的拟合系数
% fprintf('float Tpr_fitting_factor[10][4] = {\n');
% fprintf('{%ff,%ff,%ff,%ff},\n', a41(1), a41(2), a41(3), a41(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a42(1), a42(2), a42(3), a42(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a43(1), a43(2), a43(3), a43(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a44(1), a44(2), a44(3), a44(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a45(1), a45(2), a45(3), a45(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a46(1), a46(2), a46(3), a46(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a47(1), a47(2), a47(3), a47(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a48(1), a48(2), a48(3), a48(4));
% fprintf('{%ff,%ff,%ff,%ff},\n', a49(1), a49(2), a49(3), a49(4));
% fprintf('{%ff,%ff,%ff,%ff}\n', a410(1), a410(2), a410(3), a410(4));
% fprintf('};\n');
%  
% 
% toc



















% x0=leg;              %步长为0.1
% y11=polyval(a11,x0);          %返回值y0是对应于x0的函数值
% y12=polyval(a12,x0);          %返回值y0是对应于x0的函数值
% y13=polyval(a13,x0);          %返回值y0是对应于x0的函数值
% y14=polyval(a14,x0);          %返回值y0是对应于x0的函数值
% y15=polyval(a15,x0);          %返回值y0是对应于x0的函数值
% y16=polyval(a16,x0);          %返回值y0是对应于x0的函数值
% 
% y21=polyval(a21,x0);          %返回值y0是对应于x0的函数值
% y22=polyval(a22,x0);          %返回值y0是对应于x0的函数值
% y23=polyval(a23,x0);          %返回值y0是对应于x0的函数值
% y24=polyval(a24,x0);          %返回值y0是对应于x0的函数值
% y25=polyval(a25,x0);          %返回值y0是对应于x0的函数值
% y26=polyval(a26,x0);          %返回值y0是对应于x0的函数值
% subplot(3,4,1);plot(leg,k11,'o',x0,y11,'r');xlabel('x');ylabel('y');title('k11');
% subplot(3,4,2);plot(leg,k12,'o',x0,y12,'r');xlabel('x');ylabel('y');title('k12');
% subplot(3,4,5);plot(leg,k13,'o',x0,y13,'r');xlabel('x');ylabel('y');title('k13');
% subplot(3,4,6);plot(leg,k14,'o',x0,y14,'r');xlabel('x');ylabel('y');title('k14');
% subplot(3,4,9);plot(leg,k15,'o',x0,y15,'r');xlabel('x');ylabel('y');title('k15');
% subplot(3,4,10);plot(leg,k16,'o',x0,y16,'r');xlabel('x');ylabel('y');title('k16');
% 
% subplot(3,4,3);plot(leg,k21,'o',x0,y21,'r');xlabel('x');ylabel('y');title('k21');
% subplot(3,4,4);plot(leg,k22,'o',x0,y22,'r');xlabel('x');ylabel('y');title('k22');
% subplot(3,4,7);plot(leg,k23,'o',x0,y23,'r');xlabel('x');ylabel('y');title('k23');
% subplot(3,4,8);plot(leg,k24,'o',x0,y24,'r');xlabel('x');ylabel('y');title('k24');
% subplot(3,4,11);plot(leg,k25,'o',x0,y25,'r');xlabel('x');ylabel('y');title('k25');
% subplot(3,4,12);plot(leg,k26,'o',x0,y26,'r');xlabel('x');ylabel('y');title('k26');
% grid on;                   %添加网格线
% set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);  %将网格线变成虚线
% fprintf('fp32 a11[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a11(1),a11(2),a11(3),a11(4));
% fprintf('fp32 a12[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a12(1),a12(2),a12(3),a12(4));
% fprintf('fp32 a13[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a13(1),a13(2),a13(3),a13(4));
% fprintf('fp32 a14[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a14(1),a14(2),a14(3),a14(4));
% fprintf('fp32 a15[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a15(1),a15(2),a15(3),a15(4));
% fprintf('fp32 a16[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a16(1),a16(2),a16(3),a16(4));
% 
% fprintf('fp32 a21[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a21(1),a21(2),a21(3),a21(4));
% fprintf('fp32 a22[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a22(1),a22(2),a22(3),a22(4));
% fprintf('fp32 a23[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a23(1),a23(2),a23(3),a23(4));
% fprintf('fp32 a24[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a24(1),a24(2),a24(3),a24(4));
% fprintf('fp32 a25[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a25(1),a25(2),a25(3),a25(4));
% fprintf('fp32 a26[6] = {0,%.4f,%.4f,%.4f,%.4f};\n',a26(1),a26(2),a26(3),a26(4));
toc
