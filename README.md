# TD4-monitor
アクセスいただきましてありがとうございます。  

「CPUの創りかた」で有名なTD4のROM部分をマイコンで代用するためのプリント基板とソフトウエアです。  
本モニターは dip_factoryさんがBOOTHにて頒布されているプリント基板の  
26PIN拡張コネクタに接続することを意図して設計されています。  

TD4のROMは8P-DIPスイッチを16個並べて実現しており以下の特長があります。  
(1) ROMの全bitが目視可能  
(2) CPU動作中にROM内容の変更が可能  
(3) 電源を切っても内容が保持される  

上記(1)(2)を実現し、(3)に代わってマイコン内のEEPROMにプログラムを  
セーブ/ロード可能とする予定です。  

本モニタでは外部ROMに加えて以下の機能を追加する予定です。  
(4) 実行中のレジスタの内容を7segLEDにリアルタイム表示  
(5) レジスタ値を任意に設定（キャリーを含めた全bitをすべて1に出来ない制約有り）  
(6) シリアルコンソールの実行トレース出力  
(7) 供給クロックをコンソールから変更可能  

## 基板・部品
回路図はTD4-monitor_Schematic.pdfご参照ください。  
TD4_monitor231209.zip 私がJLCPCBに発注した基板の設計データです。  
![Front](https://github.com/Gazelle8087/TD4-monitor/blob/main/PCB_F.jpg)
![Back](https://github.com/Gazelle8087/TD4-monitor/blob/main/PCB_B.jpg)


## ソフトウエア
__ソフトウエアは正式リリース前です__

## 変更履歴
Ver0.0（2023.12.27時点公開前）
