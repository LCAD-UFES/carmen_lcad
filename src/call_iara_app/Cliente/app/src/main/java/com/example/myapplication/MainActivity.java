package com.example.myapplication;

import android.content.DialogInterface;
import android.content.SharedPreferences;
import android.support.v7.app.AlertDialog;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.text.InputType;
import android.text.format.Formatter;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.io.StringReader;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.Socket;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.List;
import java.util.Scanner;


public class MainActivity extends AppCompatActivity {



    private ArrayList<String> data = new ArrayList<String>();
    private ArrayList<String> locais = new ArrayList<String>();
    private ArrayList<String> arquivo = new ArrayList<String>();
    private String[] local;

    private String tempori;
    private String tempdest;
    private String enderecoip;
    private String enderecolocal;
    private String msg;
//    AlertDialog alertDialogStores;

    private Button botaoori;
    private Button botaodest;
    private Button botaook;
    private Button botaopeg;

    private TextView ori;
    private TextView dest;
    private TextView resposta;

    private int contador;

    private final String MY_PREFS_NAME = "CIApp.txt";
    //private Button botaodest = new Button();
    //private Button botaoook = new Button();



    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        //Separa os nomes dos locais
        // System.out.println(local[1]);

       contador = 0;
       botaoori = (Button) findViewById(R.id.but_ori);
       botaodest = (Button) findViewById(R.id.but_dest);
       botaook = (Button) findViewById(R.id.but_ok);
       botaopeg = (Button) findViewById(R.id.but_peg);

       ori = (TextView) findViewById(R.id.textView);
       dest = (TextView) findViewById(R.id.textView2);
       resposta = (TextView) findViewById(R.id.textView3);

       tempori = "";
       tempdest = "";
       enderecoip = "192.168.108.66";
       enderecolocal = "";
       while(enderecolocal.equals(""))
           enderecolocal = getLocalIpAddress();


        Conexao("3;"+enderecolocal);

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
            //int ret = 0;
            @Override
            public void onClick(View view) {
                if (ori.getText().length()<4 || dest.getText().length()<4) {
                    displayExceptionMessage("Preencha todos os campos");
                } else {
                    if(enviado) {
                        new AlertDialog.Builder(MainActivity.this)
                                .setTitle("Cancelar solicitação")
                                .setMessage("Realmente deseja cancelar?")
                                .setPositiveButton(android.R.string.yes, new DialogInterface.OnClickListener() {
                                    public void onClick(DialogInterface dialog, int whichButton) {
                                        mensagem = "1;" + tempori + ";" + tempdest + ";" + enderecolocal;
                                        Conexao(mensagem);
                                        enviado = false;
                                        botaook.setText("Solicitar");
                                    }})
                                .setNegativeButton(android.R.string.no, null).show();
                    }else {
                        mensagem = "2;" + tempori + ";" + tempdest + ";" + enderecolocal;
                        Conexao(mensagem);
                        enviado = true;
                        botaook.setText("Cancelar");
                    }
}
            }

        });

        botaopeg.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
               // while(arquivo.size()<3)
               Conexao("3;"+enderecolocal);

            }

        });



    }

    public void secreto(View v){
        contador++;
        if(5 == contador) {
            // System.out.println("aaAAAAAaaaaaaAAAAaaaaAA");
            contador = 0;

            AlertDialog.Builder builder = new AlertDialog.Builder(MainActivity.this);
            builder.setTitle("Digite o IP do servidor: ");
            final EditText input = new EditText(MainActivity.this);
            input.setInputType(InputType.TYPE_CLASS_TEXT);
            builder.setView(input);
            input.setText(getIP());
            builder.setPositiveButton("OK", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    if (!input.getText().toString().equals(""))
                        setIP(input.getText().toString());
                     //   enderecoip = input.getText().toString();
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


    public void Conexao(String msg1){
        msg = msg1;
        Thread t = new Thread(new Runnable() {
            @Override
            public void run() {

                String resposta = "";
                Socket soc = null;
                Scanner entrada = null;
                PrintWriter writer = null;
                try {

                    //   Socket soc = new Socket("10.42.0.25", 8000);
                    // Abaixo eh para quando eh utilizado o android pelo emulador ao inves do celular
                    soc = new Socket(getIP(), 8000);
                    writer = new PrintWriter(soc.getOutputStream());
                    soc.setSoTimeout(3000);
                    writer.write(msg);
                    writer.flush();
                    // System.out.println("Testando: "+msg);
                    //retorno = receberMensagem();
                    entrada = new Scanner(soc.getInputStream());
                    if(entrada.hasNext())
                        resposta = entrada.nextLine();
                    // System.out.println("Resposta: "+resposta);
                    if(msg.substring(0,5).equals("First") && arquivo.size()<3) {
                        System.out.println(msg.substring(0,5));
                        System.out.println(msg);
                        CapturaArquivo(resposta);
                    }else{
                        displayExceptionMessage(resposta);
                    }

                    soc.close();
                    entrada.close();
                    writer.close();

               } catch (SocketTimeoutException e){
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
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                //RecebeArquivo();
                if(!arquivo.isEmpty()) {
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

    }
    public String getLocalIpAddress() {
        try {
            for (Enumeration<NetworkInterface> en = NetworkInterface.getNetworkInterfaces(); en.hasMoreElements();) {
                NetworkInterface intf = en.nextElement();
                for (Enumeration<InetAddress> enumIpAddr = intf.getInetAddresses(); enumIpAddr.hasMoreElements();) {
                    InetAddress inetAddress = enumIpAddr.nextElement();
                    if (!inetAddress.isLoopbackAddress()) {
                        String ip = Formatter.formatIpAddress(inetAddress.hashCode());
                        return ip;
                    }
                }
            }
        } catch (SocketException ex) {
        }
        return null;
    }


    public String getIP(){
        SharedPreferences prefs = getSharedPreferences(MY_PREFS_NAME, MODE_PRIVATE);
        String restoredText = prefs.getString("text", null);
        String name = prefs.getString("ip", enderecoip);//"No name defined" is the default value.
        return name;
    }

    public void setIP(String nip){
        SharedPreferences.Editor editor = getSharedPreferences(MY_PREFS_NAME, MODE_PRIVATE).edit();
        editor.putString("ip", nip);
        editor.apply();
    }


    }




