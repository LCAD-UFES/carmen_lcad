package com.example.myapplication;

import android.content.DialogInterface;
import android.support.v7.app.AlertDialog;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.text.InputType;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringReader;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;


public class MainActivity extends AppCompatActivity {



    private ArrayList<String> data = new ArrayList<String>();
    private ArrayList<String> locais = new ArrayList<String>();
    String[] local;
    String tempori;
    String tempdest;
    String enderecoip;
    private ArrayList<String> arquivo = new ArrayList<String>();
//    AlertDialog alertDialogStores;

    private Button botaoori;
    private Button botaodest;
    private Button botaook;
    private Button botaopeg;
    private Button botaoip;


    private TextView ori;
    private TextView dest;
    private TextView resposta;

    //private Button botaodest = new Button();
    //private Button botaoook = new Button();



    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        //Separa os nomes dos locais
        // System.out.println(local[1]);

       botaoori = (Button) findViewById(R.id.but_ori);
       botaodest = (Button) findViewById(R.id.but_dest);
       botaook = (Button) findViewById(R.id.but_ok);
       botaopeg = (Button) findViewById(R.id.but_peg);
       botaoip = (Button) findViewById(R.id.but_ip);


        ori = (TextView) findViewById(R.id.textView);
       dest = (TextView) findViewById(R.id.textView2);
       resposta = (TextView) findViewById(R.id.textView3);

       tempori = "";
       tempdest = "";
       enderecoip = "192.168.108.78";


        //System.out.println(locais);
        botaoori.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(MainActivity.this);
                locais = (ArrayList<String>) data.clone();
                if(!tempori.equals("")){
                    locais.remove(tempori);

                }
                if(!tempdest.equals("")){
                    locais.remove(tempdest);

                }
                local = locais.toArray(new String[locais.size()]);

                builder.setTitle("Locais Origem");
                builder.setItems(local, new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        ori.setText(local[which]);
                        tempori=local[which];

                    }
                });

                AlertDialog alert = builder.create();
                alert.show();

            }
        });


        botaodest.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(MainActivity.this);
                locais = (ArrayList<String>) data.clone();
                if(!tempori.equals("")){
                    locais.remove(tempori);

                }
                if(!tempdest.equals("")){
                    locais.remove(tempdest);

                }
                local = locais.toArray(new String[locais.size()]);

                builder.setTitle("Locais Destino");
                builder.setItems(local, new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        dest.setText(local[which]);
                        tempdest=local[which];

                    }
                });
                AlertDialog alert = builder.create();
                alert.show();
            }
        });



        botaook.setOnClickListener(new View.OnClickListener() {
            boolean enviado = false;
            String mensagem = null;
            String retorno = null;
            //int ret = 0;
            @Override
            public void onClick(View view) {
                if (ori.getText().length()<4 || dest.getText().length()<4) {
                    displayExceptionMessage("Preencha todos os campos");
                } else {
                    Thread t = new Thread(new Runnable() {

                        @Override
                        public void run() {
                            try {

                                //   Socket soc = new Socket("10.42.0.25", 8000);
                                // Abaixo eh para quando eh utilizado o android pelo emulador ao inves do celular
                                Socket soc = new Socket(enderecoip, 8000);
                                PrintWriter writer = new PrintWriter(soc.getOutputStream());
                                mensagem = "666 Origem: " + tempori + "\tDestino: " + tempdest;
                                writer.write(mensagem);
                                writer.flush();
                                enviado = true;
                                //retorno = receberMensagem();
                                Scanner entrada = new Scanner(soc.getInputStream());
                                retorno = entrada.nextLine();

                                displayExceptionMessage(retorno);
                                writer.close();
                                entrada.close();
                                soc.close();

                            } catch (UnknownHostException e) {
                                // TODO Auto-generated catch block
                                e.printStackTrace();
                                final String erro = e.getMessage();
                                displayExceptionMessage(erro);
                            } catch (IOException e) {
                                // TODO Auto-generated catch block
                                final String erro = e.getMessage();
                                displayExceptionMessage(erro);
                                e.printStackTrace();
                            }

                        }
                    });
                    t.start();
                }
            }

        });

        botaopeg.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                RecebeArquivo();
                if (arquivo.size()>1) {
                    data = arquivo;
                    for (String e : data) {
                        locais.add(e.split(",")[0]);
//                        System.out.println(e.split(",")[0]);
                    }
                    data = (ArrayList<String>) locais.clone();
                    local = locais.toArray(new String[locais.size()]);
                    displayExceptionMessage("Locais capturados!");
                    botaopeg.setEnabled(false);
                }
            }

        });

        botaoip.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AlertDialog.Builder builder = new AlertDialog.Builder(MainActivity.this);
                builder.setTitle("Digite o IP do servidor: ");
                final EditText input = new EditText(MainActivity.this);
                input.setInputType(InputType.TYPE_CLASS_TEXT);
                builder.setView(input);
                input.setText(enderecoip);
                builder.setPositiveButton("OK", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        if(!input.getText().toString().equals(""))
                        enderecoip = input.getText().toString();
                        //System.out.println(enderecoip);
                    }
                });
                builder.setNegativeButton("Cancelar", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        dialog.cancel();
                    }
                });
                AlertDialog alert = builder.create();
                alert.show();

            }


        });


    }




    public void displayExceptionMessage(final String msg) {
        //Toast.makeText(this, msg, Toast.LENGTH_SHORT).show();
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                resposta.setText(msg);
            }
        });
    }


    private void RecebeArquivo(){
        Thread t = new Thread(new Runnable() {
            @Override
            public void run() {
                try {

                    //   Socket soc = new Socket("10.42.0.25", 8000);
                    // Abaixo eh para quando eh utilizado o android pelo emulador ao inves do celular
                    Socket soc = new Socket(enderecoip,8000);
                    PrintWriter writer = new PrintWriter(soc.getOutputStream());
                    writer.write("First");
                    writer.flush();
                    Scanner entrada = new Scanner(soc.getInputStream());
                    CapturaArquivo(entrada.nextLine());
                    writer.close();
                    entrada.close();
                    soc.close();

                } catch (UnknownHostException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                    final String erro = e.getMessage();
                    displayExceptionMessage(erro);
                } catch (IOException e) {
                    // TODO Auto-generated catch block
                    final String erro = e.getMessage();
                    displayExceptionMessage(erro);
                    e.printStackTrace();
                } catch(Exception e){
                    final String erro = e.getMessage();
                    displayExceptionMessage(erro);
                    e.printStackTrace();
                }

            }
        });
        t.start();
    }


    public void CapturaArquivo(String arq){

         arquivo = new ArrayList<String>(Arrays.asList(arq.split(";;")));

    }

    }


/*
    private ArrayList LerArquivo(){
    String Arquivo = "# Stop perto do LCAD\n" +
            "# RDDF_ANNOTATION_TYPE_STOP\t\t\t4\t0\t0.662\t\t7757722.8\t-363567.4\t0.0\n" +
            "\n" +
            "# Faixa da ambiental\n" +
            "RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP\t12\t0\t-0.702\t\t7757900.6\t-363596.2 \t0.0\n" +
            "RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK\t\t3\t0\t-0.702\t\t7757907.6\t-363599.6\t5.0\n" +
            "\n" +
            "# Centro de Linguas\n" +
            "RDDF_ANNOTATION_TYPE_BUMP\t\t\t6\t0\t-2.480828\t7757771.560955\t-363827.314529\t-1.501680\n" +
            "RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP\t12\t0\t3.137\t\t7757336.46\t-364101.10\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK\t\t3\t0\t2.404\t\t7757326.88\t-364101.69\t8.0\n" +
            "RDDF_ANNOTATION_TYPE_BUMP\t\t\t6\t0\t-2.479902\t7757720.000000\t-363872.400000\t-1.709014\n" +
            "\n" +
            "# Faixa fundacao apos ADUFES\n" +
            "RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP\t12\t0\t-2.488\t\t7757597.48\t-363965.32 \t0.0\n" +
            "RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK\t\t3\t0\t-2.511\t\t7757589.25\t-363971.52\t9.0\n" +
            "\n" +
            "# Passarela principal\n" +
            "RDDF_ANNOTATION_TYPE_BUMP\t\t\t6\t0\t-2.99100\t7757355.400000 \t-364099.500000\t-1.577188\n" +
            "RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP\t12\t0\t-2.473\t\t7757757.49\t-363842.71\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK\t\t3\t0\t-2.494\t\t7757752.48\t-363847.94\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_BUMP\t\t\t6\t0\t3.019200\t7756895.287702\t-364047.095549\t-1.439133\n" +
            "# sentido oposto\n" +
            "RDDF_ANNOTATION_TYPE_BUMP\t\t\t6\t0\t-0.075\t\t7757304.6\t-364102.4\t-1.439133\n" +
            "RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP\t12\t0\t-0.075\t\t7757319.2 \t-364103.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK\t\t3\t0\t-0.075\t\t7757327.8\t-364102.4\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_BUMP\t\t\t6\t0\t0.103\t\t7757355.4\t-364102.4\t-1.439133\n" +
            "\n" +
            "# Faixa apos estacionamento Teatro\n" +
            "RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP\t12\t0\t3.052\t\t7757163.6 \t-364074.0 \t0.0\n" +
            "RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK\t\t3\t0\t3.137\t\t7757157.85\t-364074.30\t5.0\n" +
            "# sentido oposto\n" +
            "RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP\t12\t0\t-0.100\t\t7757151.6\t-364086.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK\t\t3\t0\t-0.100\t\t7757156.6 \t-364087.4\t5.0\n" +
            "\n" +
            "# Antes da Cancela\n" +
            "RDDF_ANNOTATION_TYPE_BUMP\t\t\t6\t0\t3.052528\t7757304.931001\t-364101.008290\t-1.616369\n" +
            "RDDF_ANNOTATION_TYPE_BUMP\t\t\t6\t0\t3.000501\t7756831.985799\t-364037.939967\t-1.551413\n" +
            "RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP\t12\t0\t3.005\t\t7756889.15\t-364044.30 \t0.0\n" +
            "RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK\t\t3\t0\t3.016\t\t7756880.63\t-364044.30\t7.0\n" +
            "RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP\t12\t0\t3.003\t\t7756854.80 \t-364039.90 \t0.0\n" +
            "RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK\t\t3\t0\t3.002\t\t7756844.40 \t-364039.69\t6.0\n" +
            "\n" +
            "RDDF_ANNOTATION_TYPE_BARRIER\t\t\t5\t0\t2.797000\t7756771.800000\t-364018.201699\t-0.617266\n" +
            "\n" +
            "RDDF_ANNOTATION_TYPE_BARRIER\t\t\t5\t0\t2.006084\t7756532.200506\t-363865.800359\t-0.313729\n" +
            "RDDF_ANNOTATION_TYPE_BUMP\t\t\t6\t0\t-0.142084\t7757047.400867\t-363693.954879\t-1.599702\n" +
            "RDDF_ANNOTATION_TYPE_BUMP\t\t\t6\t0\t0.539112\t7757104.608967\t-363680.801802\t-0.399843\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t\t7\t4\t0.765114\t7757165.800747\t-363643.600393\t2.236940\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t\t7\t5\t0.765114\t7757311.200747\t-363498.209393\t2.236940 \n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t\t7\t5\t0.765114\t7757764.800000 \t-363535.200000\t2.236940\n" +
            "RDDF_ANNOTATION_TYPE_STOP\t\t\t4\t0\t-1.581000\t7757498.580000  -364040.830000\t0.0\n" +
            "\n" +
            "\n" +
            "RDDF_PLACE_ESCADARIA_TEATRO\t\t\t\t\t13\t0\t2.770\t7757268.0\t-364060.6\t0.0\n" +
            "RDDF_PLACE_ESTACIONAMENTO_TEATRO_(NORTE)\t13\t0\t-2.781\t7757216.65\t-364067.00\t0.0\n" +
            "RDDF_PLACE_ESTACIONAMENTO_TEATRO_(SUL)\t\t13\t0\t2.613\t7757075.10\t-364050.49\t0.0\n" +
            "RDDF_PLACE_ARCO_E_FLECHA\t\t\t\t\t13\t0\t2.969\t7756778.0\t-363689.2\t0.0\n" +
            "RDDF_PLACE_GINASIO_ESPORTES\t\t\t\t\t13\t0\t2.969\t7756856.4\t-363707.8\t0.0\n" +
            "RDDF_PLACE_CAMPO_FUTEBOL\t\t\t\t\t13\t0\t2.969\t7756916.8\t-363715.0\t0.0\n" +
            "RDDF_PLACE_QUADRA_ESCOLA\t\t\t\t\t13\t0\t2.969\t7756981.0\t-363703.2\t0.0\n" +
            "RDDF_PLACE_FUNDOS_R.U.\t\t\t\t\t\t13\t0\t0.615\t7757094.6\t-363725.4\t0.0\n" +
            "RDDF_PLACE_LAGO\t\t\t\t\t\t\t\t13\t0\t3.1416\t7757234.1\t-363621.5\t0.0\n" +
            "RDDF_PLACE_PLANETARIO\t\t\t\t\t\t13\t0\t-2.300\t7757254.6\t-363513.4\t0.0\n" +
            "RDDF_PLACE_ESTACIONAMENTO_PLANETARIO\t\t13\t0\t3.1416\t7757308.5\t-363480.5\t0.0\n" +
            "RDDF_PLACE_AUDITORIO_CCJE\t\t\t\t\t13\t0\t-2.500\t7757298.7\t-363539.4\t0.0\n" +
            "RDDF_PLACE_ESTACIONAMENTO_CCJE\t\t\t\t13\t0\t-2.500\t7757407.7\t-363571.2\t0.0\n" +
            "RDDF_PLACE_QUIMICA_PETROLEO\t\t\t\t\t13\t0\t-2.900\t7757582.8\t-363622.6\t0.0\n" +
            "RDDF_PLACE_CT-XII\t\t\t\t\t\t\t13\t0\t3.1416\t7757658.2\t-363674.0\t0.0\n" +
            "RDDF_PLACE_LCAD\t\t\t\t\t\t\t\t13\t0\t3.1416\t7757685.2\t-363619.6\t0.0\n" +
            "RDDF_PLACE_CT-XI\t\t\t\t\t\t\t13\t0\t3.1416\t7757766.2\t-363561.0\t0.0\n" +
            "RDDF_PLACE_CT-VI\t\t\t\t\t\t\t13\t0\t3.1416\t7757825.0\t-363581.0\t0.0\n" +
            "RDDF_PLACE_CANTINA_CT\t\t\t\t\t\t13\t0\t3.1416\t7757848.8\t-363614.0\t0.0\n" +
            "RDDF_PLACE_CT-IV\t\t\t\t\t\t\t13\t0\t3.1416\t7757782.0\t-363770.2\t0.0\n" +
            "RDDF_PLACE_NTI\t\t\t\t\t\t\t\t13\t0\t3.1416\t7757731.4\t-363793.6\t0.0\n" +
            "RDDF_PLACE_NUCLEO_LINGUAS\t\t\t\t\t13\t0\t3.1416\t7757750.4\t-363876.4\t0.0\n" +
            "RDDF_PLACE_ESTACIONAMENTO_REITORIA\t\t\t13\t0\t-2.500\t7757390.0\t-364033.2\t0.0\n" +
            "\n" +
            "# bifurcacao do LCAD (straight)\n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t20\t0.649\t\t7757673.6\t-363604.8\t0.0\n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t21\t0.649\t\t7757673.6\t-363604.8\t0.0\n" +
            "\n" +
            "# bifurcacao 1 do estacionamento da Cantina do CT (straight)\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t20\t-0.744\t\t7757858.0\t-363558.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t21\t-0.744\t\t7757864.0\t-363564.2\t0.0\n" +
            "# bifurcacao 1 do estacionamento da Cantina do CT (right)\n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t16\t-0.744\t\t7757858.0\t-363558.6\t-0.179\n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t21\t-2.315\t\t7757858.4\t-363566.4\t-0.179\n" +
            "\n" +
            "# juncao rotatoria norte Fernando Ferrari\n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t20\t-2.528\t\t7757529.0\t-364013.2 \t0.0\n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t21\t-2.528\t\t7757516.8\t-364020.4 \t0.0\n" +
            "# bifurcacao 1 rotatoria norte Fernando Ferrari (straight)\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t20\t-2.484\t\t7757503.0\t-364030.8\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t21\t-2.484\t\t7757495.4\t-364037.4\t0.0\n" +
            "# bifurcacao 1 rotatoria norte Fernando Ferrari (left)\n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t17\t-2.484\t\t7757502.8\t-364030.8\t0.064\n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t21\t-1.195\t\t7757498.8\t-364048.8\t0.064\n" +
            "# bifurcacao 2 rotatoria norte Fernando Ferrari (straight)\n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t20\t-1.195\t\t7757498.8\t-364048.8\t0.0\n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t21\t-1.195\t\t7757506.51\t-364065.37\t0.0\n" +
            "\n" +
            "# bifurcacao rotatoria estacionamento Teatro (straight/right)\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t20\t3.104\t\t7757143.42\t-364075.50\t-0.009\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t21\t2.980\t\t7757124.26\t-364071.90\t-0.009\n" +
            "\n" +
            "# bifurcacao cancela 1 sul (straight/right)\n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t20\t2.983\t\t7756827.63\t-364035.30 \t-0.0055\n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t21\t2.834\t\t7756804.6\t-364029.6\t-0.0055\n" +
            "\n" +
            "# bifurcacao cancela 2 sul (straight/right) / rotatoria sul Fernando Ferrari (left)  \n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t16\t2.569\t\t7756556.8\t-363898.8\t0.0 \n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t21\t2.569\t\t7756556.8\t-363898.8\t0.0 \n" +
            "\n" +
            "# bifurcacao da Caixa Economica (straight)\n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t20\t-0.143\t\t7756975.8\t-363684.6\t0.0\n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t21\t-0.143\t\t7756975.8\t-363684.6\t0.0\n" +
            "\n" +
            "# bifurcacao do RU (left)\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t17\t-0.136\t\t7757069.4\t-363697.2\t0.045\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t21\t0.543\t\t7757081.8\t-363693.8\t0.045\n" +
            "\n" +
            "# bifurcacao 1 do estacionamento do CCJE (straight)\n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t20\t-0.959\t\t7757414.0\t-363547.2\t0.0\n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t21\t-0.959\t\t7757414.0\t-363547.2\t0.0\n" +
            "\n" +
            "# bifurcacao 2 do estacionamento do CCJE (straight)\n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t20\t-0.879\t\t7757477.4 \t-363629.6\t0.0\n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t21\t-0.879\t\t7757477.4 \t-363629.6\t0.0\n" +
            "\n" +
            "# juncao do CCJE\n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t20\t0.458\t\t7757617.5\t-363645.5\t0.0 \n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t21\t0.458\t\t7757617.5\t-363645.5\t0.0 \n" +
            "\n" +
            "# bifurcacao do CT-X (straight)\n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t20\t0.631\t\t7757649.8\t-363623.2\t0.0\n" +
            "#RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t21\t0.631\t\t7757649.8\t-363623.2\t0.0\n" +
            "\n" +
            "\n" +
            "RDDF_ANNOTATION_TYPE_BARRIER\t\t\t5\t0\t-1.16700\t7757517.800000\t-364098.400000\t0.0\n" +
            "\n" +
            "# Ida guarapari\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.176615\t7757513.203332\t-364138.844742\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t3.016           7757280.400000\t-364123.200000\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t3.024\t\t7757167.800000\t-364109.800000\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t3.024\t\t7756854.6\t-364067.0\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.907\t\t7756152.8 \t-363808.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.768 \t\t7756025.0       -363855.0\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.768 \t\t7756025.0       -363855.0\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.768 \t\t7755797.4 \t-363948.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.768 \t\t7755667.0 \t-363996.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.768 \t\t7755523.4\t-364052.4\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.768 \t\t7755394.8\t-364103.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.768 \t\t7755255.2\t-364158.8\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.768 \t\t7755187.8\t-364186.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.768 \t\t7755073.0\t-364231.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.768 \t\t7754822.0\t-364331.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.768 \t\t7754674.6\t-364388.5\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.776 \t\t7754605.4\t-364415.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.776 \t\t7754479.0\t-364465.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.776 \t\t7754262.4\t-364551.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.759 \t\t7754167.8\t-364588.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.759 \t\t7753989.4 \t-364658.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.759 \t\t7753895.2\t-364696.4\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.759 \t\t7753854.8\t-364714.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.759 \t\t7753642.2\t-364754.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.759 \t\t7753543.4 \t-3647036.4\t0.0\n" +
            "\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.120\t  \t7750129.8\t-365146.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.120\t  \t7750102.0 \t-365061.0\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.120\t  \t7750097.8 \t-364971.2\t0.0\n" +
            "\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t2.001           7749805.4 \t-364536.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t1.998           7749775.2 \t-364471.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t1.998           7749655.4 \t-364195.8\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t1.998           7749582.8 \t-364011.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t1.998           7749542.0 \t-363870.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t1.998           7749525.4 \t-363799.8\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t1.998           7749463.6 \t-363455.8\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t1.998           7749438.2 \t-363244.0\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t1.998           7749429.4 \t-363140.4\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t1.998           7749618.2 \t-362703.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t1.998           7747655.8 \t-364264.0\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t1.998           7747630.4 \t-364156.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t1.998           7746565.8 \t-363612.4\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t2.145           7745755.4 \t-362818.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t2.932           7745579.2 \t-362810.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t2.932           7741675.2\t-361349.0\t0.0\n" +
            "\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.312832\t7757498.447006\t-364158.309644\t6.671012\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t\t7\t8\t-3.089000\t7757474.400000\t-364152.000000\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t3.026305\t7757259.061873\t-364123.043010\t6.113251\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t3.026305\t7757255.693847\t-364127.395760\t3.685989\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t3.026635\t7757151.178550\t-364106.399197\t5.147707\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t3.026635\t7757151.190923\t-364109.187887\t5.410441\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t3.008768\t7756826.814129\t-364065.824449\t5.986342\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t\t7\t8\t3.013\t\t7756994.8 \t-364086.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t3.008768\t7756827.259789\t-364062.752097\t5.658684\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-3.034066\t7756140.674064\t-363810.198169\t6.496273\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-3.034066\t7756139.760039\t-363801.640998\t5.799623\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.734121\t7756013.379345\t-363861.581190\t6.566540\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.734121\t7756013.386537\t-363865.780884\t4.144194\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.910539\t7755791.839003\t-363944.444649\t2.635340\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.910539\t7755790.2\t-363945.0\t7.023300\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.774746\t7755655.429854\t-364007.073436\t3.037819\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.805626\t7755521.459383\t-364057.664763\t3.150335\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.809264\t7755504.126879\t-364056.980693\t6.142509\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.717880\t7755375.442586\t-364112.227476\t6.337398\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t\t7\t8\t-2.752000\t7755330.4\t-364128.8\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.829537\t7755247.717614\t-364155.989259\t5.865737\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.827781\t7755245.400450\t-364163.878200\t3.997643\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.706927\t7755182.781123\t-364193.007358\t5.378651\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.706927\t7755188.951100\t-364199.605499\t2.936681\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.804779\t7755057.967971\t-364234.577990\t6.201074\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.804779\t7755061.764464\t-364241.206214\t3.829293\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.883818\t7754818.404358\t-364322.577421\t3.528960\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.886106\t7754807.900917\t-364332.760133\t5.658423\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.931084\t7754660.937539\t-364389.773217\t6.472678\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.742937\t7754587.300959\t-364424.400793\t6.805501\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.757416\t7754468.445851\t-364470.543084\t5.735607\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.757416\t7754470.395784\t-364474.660680\t3.285013\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.761062\t7754246.064601\t-364558.959987\t5.850263\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.761062\t7754247.801025\t-364563.069399\t3.674604\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.759259\t7754140.923330\t-364599.911099\t6.724307\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.767540\t7754144.730591\t-364602.817315\t2.702172\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.750767\t7753978.868718\t-364665.246868\t6.136041\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.734906\t7753884.002944\t-364704.606030\t6.769369\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.734906\t7753885.644168\t-364708.321322\t3.729662\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.826603\t7753834.532681\t-364722.519478\t6.009054\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.826603\t7753833.820955\t-364726.217744\t3.431215\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.751953\t7753841.892034\t-364714.278589\t6.050023\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t16\t-2.758\t\t7753672.4\t-364740.3\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHTPLACE\t\t8\t0\t2.564\t\t7753542.936453\t-364703.380738\t4.175933\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t2.5840          7753558.6 \t-364708.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t\t8\t0\t-2.786\t\t7753619.0\t-364759.2\t5.031814\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t\t9\t16\t-2.221\t\t7753492.0\t-364779.0\t0.0\n" +
            "\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t6\t-2.00\t\t7753464.6\t -364818.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t8\t-2.015\t\t7753405.4\t -364951.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t-2.116\t  \t7750323.6\t -365482.0\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t2.116147\t7750317.0\t -365471.243164\t5.953590\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t7\t2.085\t\t7750155.0\t -365190.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t2.085\t  \t7750131.6\t -365148.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t2.121\t\t7750126.2 \t -365137.0\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t1.6210\t  \t7750101.6\t -365062.8\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t1.6280\t\t7750103.871692\t -365043.438733\t6.388462\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t1.614410\t7750098.061680\t -365043.076869\t6.348047\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t1.640\t  \t7750096.6\t -364969.0\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t1.720\t\t7750092.515923\t -364958.963013\t6.281333\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t-3.083\t  \t7749993.8\t -364907.8\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t-3.112\t\t7749969.8\t -364904.2\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t3.001\t  \t7749585.8\t -364847.8\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t3.001\t\t7749572.8\t -364846.0\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t3.001\t  \t7749517.0\t -364837.8\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t3.001\t\t7749507.0\t -364837.0\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t3.010\t  \t7749223.6\t -364796.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t3.010\t\t7749203.2\t -364793.6\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t3.002\t  \t7749031.6\t -364769.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t3.002\t\t7749019.0\t -364767.0\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t3.002\t  \t7748891.2\t -364748.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t3.002\t\t7748881.6\t -364747.4\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t3.002\t  \t7748817.4\t -364736.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t3.002\t\t7748799.8\t -364734.0\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t3.002\t  \t7748482.8\t -364684.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t3.002\t\t7748469.4\t -364683.2\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t3.002\t  \t7748277.0\t -364654.0\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t3.002\t\t7748260.2\t -364651.2\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t2.985\t  \t7748179.8\t -364639.8\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t2.985\t\t7748167.4\t -364637.4\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t2.474\t  \t7747764.8\t -364392.0\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t2.474\t\t7747754.4\t -364384.4\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t2.430\t  \t7747628.6\t -364281.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t2.430\t\t7747616.6\t -364272.0\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t2.654\t  \t7747540.0\t -364221.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t2.654\t\t7747530.0\t -364216.2\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t2.681\t  \t7746534.2\t -363711.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t2.681\t\t7746522.4\t -363704.0\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t2.681           7746320.8\t -363602.4\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t2.681\t\t7746309.6\t -363594.8\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t2.698           7745470.0\t -363179.0\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t2.698\t\t7745459.0\t -363172.8\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t2.439           7744641.6\t -362734.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t2.439\t\t7744634.6\t -362725.8\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t2.439           7741637.8\t -361372.4\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t2.439\t\t7741621.8\t -361370.8\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t2.989           7741214.6\t -361302.8\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t2.989\t\t7741197.4\t -361300.2\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t2.991           7740703.6\t -361218.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t2.991\t\t7740684.0\t -361216.2\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t2.985           7740452.0\t -361178.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t2.985\t\t7740436.4\t -361175.0\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t2.985           7739766.6\t -361065.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t2.985\t\t7739748.6\t -361061.4\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t2.985           7731811.2\t -357753.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t2.985\t\t7731790.6\t -357754.6\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t6\t2.395\t\t7727953.6\t -354804.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t8\t2.395\t\t7727716.2\t -354667.0\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t7\t2.550\t\t7727246.2\t -354331.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t8\t2.395\t\t7727716.2\t -354667.0\t0.0\n" +
            "\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t7\t2.872\t\t7720544.8\t -351224.8\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t7\t2.758\t\t7720108.2\t -351071.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t7\t2.758\t\t7719472.4\t -350782.8\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t8\t2.758\t\t7719363.2\t -350138.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t9\t17\t1.507\t\t7717975.8\t -341732.4\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t1.535           7717979.6\t -341474.0\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t2.353\t\t7717970.4\t -341464.4\t6.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t9\t16\t3.111\t\t7717852.4\t -341413.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t7\t2.465\t\t7705785.4\t -338412.4\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t9\t16\t-1.020\t\t7705674.2\t -338461.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t8\t-0.955\t\t7705751.6\t -338582.0\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t7\t-1.116\t\t7706016.6\t -339219.0\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN\t9\t16\t-1.059\t\t7706034.4\t -339252.8\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t5\t-1.059\t\t7706055.4\t -339329.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t3\t-1.478\t\t7706078.2\t -339564.4\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_BUMP\t\t6\t0\t-1.669\t\t7706072.8\t -339692.0\t0.0\n" +
            "PLACE\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t4\t 1.313\t\t7741000.2\t -360938.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t4\t-0.199\t\t7741535.39\t -361148.91\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t3\t-2.037\t\t7741575.26\t -361157.20\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t4\t 2.933\t\t7740726.4\t -360794.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t3\t1.433\t\t7740699.8\t -360785.2\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t4\t2.830\t\t7741026.08\t -360927.37\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t4\t-1.952\t\t7740998.6\t -361017.4\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t3\t2.782\t\t7740979.4\t -361028.6\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_STOP\t\t4\t0\t1.159 \t\t7740978.8\t -361013.4\t0.0\n" +
            "\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t3\t-0.225\t\t7383584.0\t -324695.0\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t3\t0.483\t\t7383549.77\t -324708.57\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t2\t2.855\t\t7383574.0\t -324702.8 \t0.0\n" +
            "RDDF_ANNOTATION_TYPE_BUMP\t\t6\t0\t2.818\t\t7383568.2\t -324700.8\t0.0\n" +
            "\n" +
            "# CENPES\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t2\t-1.534\t\t7471590.50\t-681590.14\t2.236940\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t2\t1.534\t\t7471590.50\t-681590.14\t2.236940\n" +
            "RDDF_ANNOTATION_TYPE_SPEED_LIMIT\t7\t2\t1.019\t\t7471507.73\t-681475.81\t2.236940\n" +
            "RDDF_ANNOTATION_TYPE_STOP\t\t4\t0\t-0.991 \t\t7471558.24\t-681532.40\t0.0\n" +
            "\n" +
            "# Casa Firjan\n" +
            "#RDDF_ANNOTATION_TYPE_STOP\t\t4\t0\t0.354 \t\t7460861.13\t-685656.30\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_STOP\t\t4\t0\t-1.214 \t\t7460844.71\t-685653.73\t0.0\n" +
            "\n" +
            "# Vale Centro IA\n" +
            "RDDF_ANNOTATION_TYPE_STOP\t\t4\t0\t2.485 \t\t7758567.6\t-368856.0\t0.0\n" +
            "\n" +
            "# Aeroporto Vila Velha\n" +
            "RDDF_ANNOTATION_TYPE_STOP\t\t4\t0\t-1.977 \t\t7740999.61\t-361012.60\t0.0\n" +
            "\n" +
            "####################################################################################################################################\n" +
            "# DANTE MICHELINI\n" +
            "####################################################################################################################################\n" +
            "## Todas as anotacoes foram feitas usando o log da pista do meio, da Dante Michelini. Ver o 'process-dante-michelini-playback.ini'.\n" +
            "\n" +
            "#### Depois da rotatoria\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-2.114 7755090.26 -365142.86\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t-2.151 7755080.131193 -365162.185143 4.407651 # Cluster Size 139\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHTPLACE\t8\t0\t-2.151 7755076.131051 -365160.463434 2.622644 # Cluster Size 86\n" +
            "\n" +
            "#### Antes da ponte (start message: 17000)\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t\t10\t0\t-0.605 7755192.99 -365246.71\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t-0.611514\t7755211.225190 -365258.373580 5.338932 # Cluster Size 15 # TL do topo\n" +
            "\n" +
            "#### Entrada 1 (start message: 30000 or 33000)\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t0.516\t7755674.52 -365316.52\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t0.480\t7755685.891737 -365308.139106 6.068098 # Reta, semaforo da esquerda\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t0.480\t7755687.106634 -365310.716048 5.474941 # Reta, semaforo da direita\n" +
            "\n" +
            "#### Primeiro apos a entrada 1 (start message: 40000)\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t0.225\t 7755813.76\t-365264.87 \t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t0.222181 7755832.191834 -365259.886111 5.370898 # Cluster Size 23\n" +
            "\n" +
            "#### Segundo apos a entrada 1 (start message: 46000)\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t0.165\t 7756110.07\t-365203.24\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t0.108000 7756122.632415 -365199.960211 5.538647 # Cluster Size 30 # Semaforo do topo\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t0.108000 7756122.595923 -365195.762655 2.975850 # Cluster Size 11 # Semaforo da esquerda\n" +
            "\n" +
            "#### Terceiro apos a entrada 1 (start message: 49000)\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t-0.036\t  7756218.42\t -365201.03\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t-0.037863 7756231.882346 -365200.229811 5.621069 # Cluster Size 8\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t-0.037863 7756240.513793 -365195.142497 3.428813 # Cluster Size 5\n" +
            "\n" +
            "#### Quarto apos a entrada 1 (start message: 60000)\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t-0.270\t  7756450.22\t -365236.22\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t-0.272671 7756465.499411 -365235.303176 2.855753 # Pick. Semaforo da esquerda.\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t-0.273557 7756464.521473 -365239.551520 5.465629 # Cluster Size 64\n" +
            "\n" +
            "##############checar 67000\n" +
            "#### Entrada 2 (start message: 65000PLACE)\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t-0.464\t7756672.45\t-365339.38\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t-0.470\t7756692.198848\t-365341.672691 2.362598 # RUIM! Pista do meio, semaforo da esquerda\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t-0.470\t7756682.572092\t-365342.941978 5.913052 # Reta, semaforo do topo\n" +
            "\n" +
            "#### Primeiro apos a entrada 2 (start message: 69000)\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t-0.486\t  7756792.75\t -365401.90\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t-0.486213 7756808.763967 -365412.008329 5.714606 # Cluster Size 30 # Semaforo do topo\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t-0.483710 7756807.332519 -365416.229542 2.517914 # Pick. Semaforo da direita.\n" +
            "\n" +
            "#### Segundo apos a entrada 2 (startPLACE message: 73000)\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t-0.500\t  7756942.39\t -365483.52\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t-0.501491 7756959.355512 -365491.680791 5.252125 # Cluster Size 6\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t-0.501491 7756958.621225 -365493.296173 5.124078 # Cluster Size 34\n" +
            "\n" +
            "#### Entrada 3 (start message: 79000)\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t-0.733\t7757146.51\t-365633.40\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t-0.730\t7757154.814873\t-365638.142271\t6.009828 # Reta, semaforo do topo\n" +
            "\n" +
            "#### Entrada 4 (start message: 87000)\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t-0.724\t  7757327.47\t -365797.13\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t-0.796187 7757336.759752 -365806.718414 5.322179 # Cluster Size 13\n" +
            "\n" +
            "#### Apos a entrada 4 (start message: 98000)\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP\t10\t0\t-0.812\t  7757607.70\t -366102.70\t0.0\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t-0.794352 7757618.023093 -366120.453794 4.951004 # Cluster Size 90 # Topo\n" +
            "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t-0.810481 7757616.311472 -366108.225414 2.593993 # Pick. Semaforo da esquerda.\n";

        BufferedReader reader = new BufferedReader(new StringReader(Arquivo));
        String str;
        ArrayList<String> lista = new ArrayList<String>();
        try {
            while ((str = reader.readLine()) != null){
                if(str.substring(0, Math.min(str.length(), 10)).equals("RDDF_PLACE")){
                    lista.add(str);
                }
            }


        }catch(IOException e){
            e.printStackTrace();
        }

        return lista;
    }

*/



