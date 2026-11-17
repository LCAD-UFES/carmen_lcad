<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="9.6.2">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="88" name="SimResults" color="9" fill="1" visible="yes" active="yes"/>
<layer number="89" name="SimProbes" color="9" fill="1" visible="yes" active="yes"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="con-garry" urn="urn:adsk.eagle:library:147">
<description>&lt;b&gt;Pin Header Connectors&lt;/b&gt;&lt;p&gt;
&lt;a href="www.mpe-connector.de"&gt;Menufacturer&lt;/a&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="332-48" urn="urn:adsk.eagle:footprint:6807/1" library_version="2">
<description>&lt;b&gt;48 Pin - 2mm Dual Row&lt;/b&gt;&lt;p&gt;
Source: www.mpe-connector.de / garry_shortform_2012.pdf</description>
<wire x1="-23.85" y1="-1.9" x2="23.85" y2="-1.9" width="0.2032" layer="21"/>
<wire x1="23.85" y1="-1.9" x2="23.85" y2="-0.4" width="0.2032" layer="21"/>
<wire x1="23.85" y1="0.4" x2="23.85" y2="1.9" width="0.2032" layer="21"/>
<wire x1="23.85" y1="1.9" x2="-23.85" y2="1.9" width="0.2032" layer="21"/>
<wire x1="-23.85" y1="1.9" x2="-23.85" y2="0.4" width="0.2032" layer="21"/>
<wire x1="-23.85" y1="-0.4" x2="-23.85" y2="-1.9" width="0.2032" layer="21"/>
<wire x1="-23.85" y1="0.4" x2="-23.85" y2="-0.4" width="0.2032" layer="21" curve="-129.185"/>
<wire x1="23.85" y1="-0.4" x2="23.85" y2="0.4" width="0.2032" layer="21" curve="-129.185"/>
<wire x1="-23" y1="1" x2="-23" y2="-6" width="0.55" layer="51"/>
<wire x1="-21" y1="1" x2="-21" y2="-6" width="0.55" layer="51"/>
<wire x1="-19" y1="1" x2="-19" y2="-6" width="0.55" layer="51"/>
<wire x1="-17" y1="1" x2="-17" y2="-6" width="0.55" layer="51"/>
<wire x1="-15" y1="1" x2="-15" y2="-6" width="0.55" layer="51"/>
<wire x1="-13" y1="1" x2="-13" y2="-6" width="0.55" layer="51"/>
<wire x1="-11" y1="1" x2="-11" y2="-6" width="0.55" layer="51"/>
<wire x1="-9" y1="1" x2="-9" y2="-6" width="0.55" layer="51"/>
<wire x1="-7" y1="1" x2="-7" y2="-6" width="0.55" layer="51"/>
<wire x1="-5" y1="1" x2="-5" y2="-6" width="0.55" layer="51"/>
<wire x1="-3" y1="1" x2="-3" y2="-6" width="0.55" layer="51"/>
<wire x1="-1" y1="1" x2="-1" y2="-6" width="0.55" layer="51"/>
<wire x1="1" y1="1" x2="1" y2="-6" width="0.55" layer="51"/>
<wire x1="3" y1="1" x2="3" y2="-6" width="0.55" layer="51"/>
<wire x1="5" y1="1" x2="5" y2="-6" width="0.55" layer="51"/>
<wire x1="7" y1="1" x2="7" y2="-6" width="0.55" layer="51"/>
<wire x1="9" y1="1" x2="9" y2="-6" width="0.55" layer="51"/>
<wire x1="11" y1="1" x2="11" y2="-6" width="0.55" layer="51"/>
<wire x1="13" y1="1" x2="13" y2="-6" width="0.55" layer="51"/>
<wire x1="15" y1="1" x2="15" y2="-6" width="0.55" layer="51"/>
<wire x1="17" y1="1" x2="17" y2="-6" width="0.55" layer="51"/>
<wire x1="19" y1="1" x2="19" y2="-6" width="0.55" layer="51"/>
<wire x1="21" y1="1" x2="21" y2="-6" width="0.55" layer="51"/>
<wire x1="23" y1="1" x2="23" y2="-6" width="0.55" layer="51"/>
<pad name="1" x="-23" y="-1" drill="0.9" diameter="1.27"/>
<pad name="2" x="-23" y="1" drill="0.9" diameter="1.27"/>
<pad name="3" x="-21" y="-1" drill="0.9" diameter="1.27"/>
<pad name="4" x="-21" y="1" drill="0.9" diameter="1.27"/>
<pad name="5" x="-19" y="-1" drill="0.9" diameter="1.27"/>
<pad name="6" x="-19" y="1" drill="0.9" diameter="1.27"/>
<pad name="7" x="-17" y="-1" drill="0.9" diameter="1.27"/>
<pad name="8" x="-17" y="1" drill="0.9" diameter="1.27"/>
<pad name="9" x="-15" y="-1" drill="0.9" diameter="1.27"/>
<pad name="10" x="-15" y="1" drill="0.9" diameter="1.27"/>
<pad name="11" x="-13" y="-1" drill="0.9" diameter="1.27"/>
<pad name="12" x="-13" y="1" drill="0.9" diameter="1.27"/>
<pad name="13" x="-11" y="-1" drill="0.9" diameter="1.27"/>
<pad name="14" x="-11" y="1" drill="0.9" diameter="1.27"/>
<pad name="15" x="-9" y="-1" drill="0.9" diameter="1.27"/>
<pad name="16" x="-9" y="1" drill="0.9" diameter="1.27"/>
<pad name="17" x="-7" y="-1" drill="0.9" diameter="1.27"/>
<pad name="18" x="-7" y="1" drill="0.9" diameter="1.27"/>
<pad name="19" x="-5" y="-1" drill="0.9" diameter="1.27"/>
<pad name="20" x="-5" y="1" drill="0.9" diameter="1.27"/>
<pad name="21" x="-3" y="-1" drill="0.9" diameter="1.27"/>
<pad name="22" x="-3" y="1" drill="0.9" diameter="1.27"/>
<pad name="23" x="-1" y="-1" drill="0.9" diameter="1.27"/>
<pad name="24" x="-1" y="1" drill="0.9" diameter="1.27"/>
<pad name="25" x="1" y="-1" drill="0.9" diameter="1.27"/>
<pad name="26" x="1" y="1" drill="0.9" diameter="1.27"/>
<pad name="27" x="3" y="-1" drill="0.9" diameter="1.27"/>
<pad name="28" x="3" y="1" drill="0.9" diameter="1.27"/>
<pad name="29" x="5" y="-1" drill="0.9" diameter="1.27"/>
<pad name="30" x="5" y="1" drill="0.9" diameter="1.27"/>
<pad name="31" x="7" y="-1" drill="0.9" diameter="1.27"/>
<pad name="32" x="7" y="1" drill="0.9" diameter="1.27"/>
<pad name="33" x="9" y="-1" drill="0.9" diameter="1.27"/>
<pad name="34" x="9" y="1" drill="0.9" diameter="1.27"/>
<pad name="35" x="11" y="-1" drill="0.9" diameter="1.27"/>
<pad name="36" x="11" y="1" drill="0.9" diameter="1.27"/>
<pad name="37" x="13" y="-1" drill="0.9" diameter="1.27"/>
<pad name="38" x="13" y="1" drill="0.9" diameter="1.27"/>
<pad name="39" x="15" y="-1" drill="0.9" diameter="1.27"/>
<pad name="40" x="15" y="1" drill="0.9" diameter="1.27"/>
<pad name="41" x="17" y="-1" drill="0.9" diameter="1.27"/>
<pad name="42" x="17" y="1" drill="0.9" diameter="1.27"/>
<pad name="43" x="19" y="-1" drill="0.9" diameter="1.27"/>
<pad name="44" x="19" y="1" drill="0.9" diameter="1.27"/>
<pad name="45" x="21" y="-1" drill="0.9" diameter="1.27"/>
<pad name="46" x="21" y="1" drill="0.9" diameter="1.27"/>
<pad name="47" x="23" y="-1" drill="0.9" diameter="1.27"/>
<pad name="48" x="23" y="1" drill="0.9" diameter="1.27"/>
<text x="-23.65" y="-1.75" size="0.3048" layer="21" font="vector">1</text>
<text x="-23.62" y="2.54" size="1.27" layer="25">&gt;NAME</text>
<text x="-17.27" y="2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-23.25" y1="-1.25" x2="-22.75" y2="-0.75" layer="51"/>
<rectangle x1="-23.25" y1="0.75" x2="-22.75" y2="1.25" layer="51"/>
<rectangle x1="-21.25" y1="-1.25" x2="-20.75" y2="-0.75" layer="51"/>
<rectangle x1="-21.25" y1="0.75" x2="-20.75" y2="1.25" layer="51"/>
<rectangle x1="-19.25" y1="-1.25" x2="-18.75" y2="-0.75" layer="51"/>
<rectangle x1="-19.25" y1="0.75" x2="-18.75" y2="1.25" layer="51"/>
<rectangle x1="-17.25" y1="-1.25" x2="-16.75" y2="-0.75" layer="51"/>
<rectangle x1="-17.25" y1="0.75" x2="-16.75" y2="1.25" layer="51"/>
<rectangle x1="-15.25" y1="-1.25" x2="-14.75" y2="-0.75" layer="51"/>
<rectangle x1="-15.25" y1="0.75" x2="-14.75" y2="1.25" layer="51"/>
<rectangle x1="-13.25" y1="-1.25" x2="-12.75" y2="-0.75" layer="51"/>
<rectangle x1="-13.25" y1="0.75" x2="-12.75" y2="1.25" layer="51"/>
<rectangle x1="-11.25" y1="-1.25" x2="-10.75" y2="-0.75" layer="51"/>
<rectangle x1="-11.25" y1="0.75" x2="-10.75" y2="1.25" layer="51"/>
<rectangle x1="-9.25" y1="-1.25" x2="-8.75" y2="-0.75" layer="51"/>
<rectangle x1="-9.25" y1="0.75" x2="-8.75" y2="1.25" layer="51"/>
<rectangle x1="-7.25" y1="-1.25" x2="-6.75" y2="-0.75" layer="51"/>
<rectangle x1="-7.25" y1="0.75" x2="-6.75" y2="1.25" layer="51"/>
<rectangle x1="-5.25" y1="-1.25" x2="-4.75" y2="-0.75" layer="51"/>
<rectangle x1="-5.25" y1="0.75" x2="-4.75" y2="1.25" layer="51"/>
<rectangle x1="-3.25" y1="-1.25" x2="-2.75" y2="-0.75" layer="51"/>
<rectangle x1="-3.25" y1="0.75" x2="-2.75" y2="1.25" layer="51"/>
<rectangle x1="-1.25" y1="-1.25" x2="-0.75" y2="-0.75" layer="51"/>
<rectangle x1="-1.25" y1="0.75" x2="-0.75" y2="1.25" layer="51"/>
<rectangle x1="0.75" y1="-1.25" x2="1.25" y2="-0.75" layer="51"/>
<rectangle x1="0.75" y1="0.75" x2="1.25" y2="1.25" layer="51"/>
<rectangle x1="2.75" y1="-1.25" x2="3.25" y2="-0.75" layer="51"/>
<rectangle x1="2.75" y1="0.75" x2="3.25" y2="1.25" layer="51"/>
<rectangle x1="4.75" y1="-1.25" x2="5.25" y2="-0.75" layer="51"/>
<rectangle x1="4.75" y1="0.75" x2="5.25" y2="1.25" layer="51"/>
<rectangle x1="6.75" y1="-1.25" x2="7.25" y2="-0.75" layer="51"/>
<rectangle x1="6.75" y1="0.75" x2="7.25" y2="1.25" layer="51"/>
<rectangle x1="8.75" y1="-1.25" x2="9.25" y2="-0.75" layer="51"/>
<rectangle x1="8.75" y1="0.75" x2="9.25" y2="1.25" layer="51"/>
<rectangle x1="10.75" y1="-1.25" x2="11.25" y2="-0.75" layer="51"/>
<rectangle x1="10.75" y1="0.75" x2="11.25" y2="1.25" layer="51"/>
<rectangle x1="12.75" y1="-1.25" x2="13.25" y2="-0.75" layer="51"/>
<rectangle x1="12.75" y1="0.75" x2="13.25" y2="1.25" layer="51"/>
<rectangle x1="14.75" y1="-1.25" x2="15.25" y2="-0.75" layer="51"/>
<rectangle x1="14.75" y1="0.75" x2="15.25" y2="1.25" layer="51"/>
<rectangle x1="16.75" y1="-1.25" x2="17.25" y2="-0.75" layer="51"/>
<rectangle x1="16.75" y1="0.75" x2="17.25" y2="1.25" layer="51"/>
<rectangle x1="18.75" y1="-1.25" x2="19.25" y2="-0.75" layer="51"/>
<rectangle x1="18.75" y1="0.75" x2="19.25" y2="1.25" layer="51"/>
<rectangle x1="20.75" y1="-1.25" x2="21.25" y2="-0.75" layer="51"/>
<rectangle x1="20.75" y1="0.75" x2="21.25" y2="1.25" layer="51"/>
<rectangle x1="22.75" y1="-1.25" x2="23.25" y2="-0.75" layer="51"/>
<rectangle x1="22.75" y1="0.75" x2="23.25" y2="1.25" layer="51"/>
</package>
</packages>
<packages3d>
<package3d name="332-48" urn="urn:adsk.eagle:package:6829/1" type="box" library_version="2">
<description>48 Pin - 2mm Dual Row
Source: www.mpe-connector.de / garry_shortform_2012.pdf</description>
<packageinstances>
<packageinstance name="332-48"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="MV" urn="urn:adsk.eagle:symbol:6783/1" library_version="2">
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.6096" layer="94"/>
<text x="2.54" y="-0.762" size="1.524" layer="95">&gt;NAME</text>
<text x="-0.762" y="1.397" size="1.778" layer="96">&gt;VALUE</text>
<pin name="S" x="-2.54" y="0" visible="off" length="short" direction="pas"/>
</symbol>
<symbol name="M" urn="urn:adsk.eagle:symbol:6785/1" library_version="2">
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.6096" layer="94"/>
<text x="2.54" y="-0.762" size="1.524" layer="95">&gt;NAME</text>
<pin name="S" x="-2.54" y="0" visible="off" length="short" direction="pas"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="332-48" urn="urn:adsk.eagle:component:6856/2" prefix="X" library_version="2">
<description>&lt;b&gt;48 Pin - 2mm Dual Row&lt;/b&gt;&lt;p&gt;
Source: www.mpe-connector.de / garry_shortform_2012.pdf</description>
<gates>
<gate name="-1" symbol="MV" x="-10.16" y="30.48" addlevel="always"/>
<gate name="-2" symbol="MV" x="10.16" y="30.48" addlevel="always"/>
<gate name="-3" symbol="M" x="-10.16" y="27.94" addlevel="always"/>
<gate name="-4" symbol="M" x="10.16" y="27.94" addlevel="always"/>
<gate name="-5" symbol="M" x="-10.16" y="25.4" addlevel="always"/>
<gate name="-6" symbol="M" x="10.16" y="25.4" addlevel="always"/>
<gate name="-7" symbol="M" x="-10.16" y="22.86" addlevel="always"/>
<gate name="-8" symbol="M" x="10.16" y="22.86" addlevel="always"/>
<gate name="-9" symbol="M" x="-10.16" y="20.32" addlevel="always"/>
<gate name="-10" symbol="M" x="10.16" y="20.32" addlevel="always"/>
<gate name="-11" symbol="M" x="-10.16" y="17.78" addlevel="always"/>
<gate name="-12" symbol="M" x="10.16" y="17.78" addlevel="always"/>
<gate name="-13" symbol="M" x="-10.16" y="15.24" addlevel="always"/>
<gate name="-14" symbol="M" x="10.16" y="15.24" addlevel="always"/>
<gate name="-15" symbol="M" x="-10.16" y="12.7" addlevel="always"/>
<gate name="-16" symbol="M" x="10.16" y="12.7" addlevel="always"/>
<gate name="-17" symbol="M" x="-10.16" y="10.16" addlevel="always"/>
<gate name="-18" symbol="M" x="10.16" y="10.16" addlevel="always"/>
<gate name="-19" symbol="M" x="-10.16" y="7.62" addlevel="always"/>
<gate name="-20" symbol="M" x="10.16" y="7.62" addlevel="always"/>
<gate name="-21" symbol="M" x="-10.16" y="5.08" addlevel="always"/>
<gate name="-22" symbol="M" x="10.16" y="5.08" addlevel="always"/>
<gate name="-23" symbol="M" x="-10.16" y="2.54" addlevel="always"/>
<gate name="-24" symbol="M" x="10.16" y="2.54" addlevel="always"/>
<gate name="-25" symbol="M" x="-10.16" y="0" addlevel="always"/>
<gate name="-26" symbol="M" x="10.16" y="0" addlevel="always"/>
<gate name="-27" symbol="M" x="-10.16" y="-2.54" addlevel="always"/>
<gate name="-28" symbol="M" x="10.16" y="-2.54" addlevel="always"/>
<gate name="-29" symbol="M" x="-10.16" y="-5.08" addlevel="always"/>
<gate name="-30" symbol="M" x="10.16" y="-5.08" addlevel="always"/>
<gate name="-31" symbol="M" x="-10.16" y="-7.62" addlevel="always"/>
<gate name="-32" symbol="M" x="10.16" y="-7.62" addlevel="always"/>
<gate name="-33" symbol="M" x="-10.16" y="-10.16" addlevel="always"/>
<gate name="-34" symbol="M" x="10.16" y="-10.16" addlevel="always"/>
<gate name="-35" symbol="M" x="-10.16" y="-12.7" addlevel="always"/>
<gate name="-36" symbol="M" x="10.16" y="-12.7" addlevel="always"/>
<gate name="-37" symbol="M" x="-10.16" y="-15.24" addlevel="always"/>
<gate name="-38" symbol="M" x="10.16" y="-15.24" addlevel="always"/>
<gate name="-39" symbol="M" x="-10.16" y="-17.78" addlevel="always"/>
<gate name="-40" symbol="M" x="10.16" y="-17.78" addlevel="always"/>
<gate name="-41" symbol="M" x="-10.16" y="-20.32" addlevel="always"/>
<gate name="-42" symbol="M" x="10.16" y="-20.32" addlevel="always"/>
<gate name="-43" symbol="M" x="-10.16" y="-22.86" addlevel="always"/>
<gate name="-44" symbol="M" x="10.16" y="-22.86" addlevel="always"/>
<gate name="-45" symbol="M" x="-10.16" y="-25.4" addlevel="always"/>
<gate name="-46" symbol="M" x="10.16" y="-25.4" addlevel="always"/>
<gate name="-47" symbol="M" x="-10.16" y="-27.94" addlevel="always"/>
<gate name="-48" symbol="M" x="10.16" y="-27.94" addlevel="always"/>
</gates>
<devices>
<device name="" package="332-48">
<connects>
<connect gate="-1" pin="S" pad="1"/>
<connect gate="-10" pin="S" pad="10"/>
<connect gate="-11" pin="S" pad="11"/>
<connect gate="-12" pin="S" pad="12"/>
<connect gate="-13" pin="S" pad="13"/>
<connect gate="-14" pin="S" pad="14"/>
<connect gate="-15" pin="S" pad="15"/>
<connect gate="-16" pin="S" pad="16"/>
<connect gate="-17" pin="S" pad="17"/>
<connect gate="-18" pin="S" pad="18"/>
<connect gate="-19" pin="S" pad="19"/>
<connect gate="-2" pin="S" pad="2"/>
<connect gate="-20" pin="S" pad="20"/>
<connect gate="-21" pin="S" pad="21"/>
<connect gate="-22" pin="S" pad="22"/>
<connect gate="-23" pin="S" pad="23"/>
<connect gate="-24" pin="S" pad="24"/>
<connect gate="-25" pin="S" pad="25"/>
<connect gate="-26" pin="S" pad="26"/>
<connect gate="-27" pin="S" pad="27"/>
<connect gate="-28" pin="S" pad="28"/>
<connect gate="-29" pin="S" pad="29"/>
<connect gate="-3" pin="S" pad="3"/>
<connect gate="-30" pin="S" pad="30"/>
<connect gate="-31" pin="S" pad="31"/>
<connect gate="-32" pin="S" pad="32"/>
<connect gate="-33" pin="S" pad="33"/>
<connect gate="-34" pin="S" pad="34"/>
<connect gate="-35" pin="S" pad="35"/>
<connect gate="-36" pin="S" pad="36"/>
<connect gate="-37" pin="S" pad="37"/>
<connect gate="-38" pin="S" pad="38"/>
<connect gate="-39" pin="S" pad="39"/>
<connect gate="-4" pin="S" pad="4"/>
<connect gate="-40" pin="S" pad="40"/>
<connect gate="-41" pin="S" pad="41"/>
<connect gate="-42" pin="S" pad="42"/>
<connect gate="-43" pin="S" pad="43"/>
<connect gate="-44" pin="S" pad="44"/>
<connect gate="-45" pin="S" pad="45"/>
<connect gate="-46" pin="S" pad="46"/>
<connect gate="-47" pin="S" pad="47"/>
<connect gate="-48" pin="S" pad="48"/>
<connect gate="-5" pin="S" pad="5"/>
<connect gate="-6" pin="S" pad="6"/>
<connect gate="-7" pin="S" pad="7"/>
<connect gate="-8" pin="S" pad="8"/>
<connect gate="-9" pin="S" pad="9"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:6829/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="X1" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X2" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X3" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X4" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X5" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X6" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X7" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X8" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X9" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X10" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X11" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X12" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X13" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X14" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X15" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X16" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X17" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X18" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X19" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X20" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X21" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X22" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X23" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X24" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X25" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X26" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
<part name="X27" library="con-garry" library_urn="urn:adsk.eagle:library:147" deviceset="332-48" device="" package3d_urn="urn:adsk.eagle:package:6829/1"/>
</parts>
<sheets>
<sheet>
<plain>
<text x="-30.48" y="121.92" size="1.778" layer="91">1</text>
<text x="-30.48" y="119.38" size="1.778" layer="91">2</text>
<text x="-30.48" y="116.84" size="1.778" layer="91">3</text>
<text x="-30.48" y="114.3" size="1.778" layer="91">4</text>
<text x="-30.48" y="111.76" size="1.778" layer="91">5</text>
<text x="-30.48" y="109.22" size="1.778" layer="91">6</text>
<text x="-30.48" y="106.68" size="1.778" layer="91">7</text>
<text x="-30.48" y="104.14" size="1.778" layer="91">8</text>
<text x="-30.48" y="101.6" size="1.778" layer="91">9</text>
<text x="-31.75" y="99.06" size="1.778" layer="91">10</text>
<text x="-31.75" y="96.52" size="1.778" layer="91">11</text>
<text x="-31.75" y="93.98" size="1.778" layer="91">12</text>
<text x="-31.75" y="91.44" size="1.778" layer="91">13</text>
<text x="-31.75" y="88.9" size="1.778" layer="91">14</text>
<text x="-31.75" y="86.36" size="1.778" layer="91">15</text>
<text x="-31.75" y="83.82" size="1.778" layer="91">16</text>
<text x="-31.75" y="81.28" size="1.778" layer="91">17</text>
<text x="-31.75" y="78.74" size="1.778" layer="91">18</text>
<text x="-31.75" y="76.2" size="1.778" layer="91">19</text>
<text x="-31.75" y="73.66" size="1.778" layer="91">20</text>
<text x="-31.75" y="71.12" size="1.778" layer="91">21</text>
<text x="-31.75" y="68.58" size="1.778" layer="91">22</text>
<text x="-31.75" y="66.04" size="1.778" layer="91">23</text>
<text x="-31.75" y="63.5" size="1.778" layer="91">24</text>
<text x="-31.75" y="60.96" size="1.778" layer="91">25</text>
<text x="-31.75" y="58.42" size="1.778" layer="91">26</text>
<text x="-31.75" y="55.88" size="1.778" layer="91">27</text>
<text x="-31.75" y="53.34" size="1.778" layer="91">28</text>
<text x="-31.75" y="50.8" size="1.778" layer="91">29</text>
<text x="-31.75" y="48.26" size="1.778" layer="91">30</text>
<text x="-31.75" y="45.72" size="1.778" layer="91">31</text>
<text x="-31.75" y="43.18" size="1.778" layer="91">32</text>
<text x="-31.75" y="40.64" size="1.778" layer="91">33</text>
<text x="-31.75" y="38.1" size="1.778" layer="91">34</text>
<text x="-31.75" y="35.56" size="1.778" layer="91">35</text>
<text x="-31.75" y="33.02" size="1.778" layer="91">36</text>
<text x="-31.75" y="30.48" size="1.778" layer="91">37</text>
<text x="-31.75" y="27.94" size="1.778" layer="91">38</text>
<text x="-31.75" y="25.4" size="1.778" layer="91">39</text>
<text x="-31.75" y="22.86" size="1.778" layer="91">40</text>
<text x="-31.75" y="20.32" size="1.778" layer="91">41</text>
<text x="-31.75" y="17.78" size="1.778" layer="91">42</text>
<text x="-31.75" y="15.24" size="1.778" layer="91">43</text>
<text x="-31.75" y="12.7" size="1.778" layer="91">44</text>
<text x="-31.75" y="10.16" size="1.778" layer="91">45</text>
<text x="-31.75" y="7.62" size="1.778" layer="91">46</text>
<text x="-31.75" y="5.08" size="1.778" layer="91">47</text>
<text x="-31.75" y="2.54" size="1.778" layer="91">48</text>
<text x="-56.515" y="121.285" size="1.778" layer="91">Vermelho direçao</text>
<text x="-52.705" y="118.745" size="1.778" layer="91">Verde direçao</text>
<text x="-50.8" y="116.205" size="1.778" layer="91">Azul direçao</text>
<text x="-51.435" y="113.665" size="1.778" layer="91">GND ODrive</text>
<text x="-51.435" y="111.125" size="1.778" layer="91">VCC ODrive</text>
<text x="-43.18" y="108.585" size="1.778" layer="91">Lora</text>
<text x="-51.435" y="100.965" size="1.778" layer="91">CAN_GND</text>
<text x="-48.895" y="98.425" size="1.778" layer="91">CAN_H</text>
<text x="-48.895" y="95.885" size="1.778" layer="91">CAN_L</text>
<text x="-53.975" y="93.345" size="1.778" layer="91">Volante_SEN</text>
<text x="-53.975" y="90.805" size="1.778" layer="91">Volante_GND</text>
<text x="-53.975" y="88.265" size="1.778" layer="91">Volante_COS</text>
<text x="-56.515" y="85.725" size="1.778" layer="91">Volante_SIGNAL</text>
<text x="-64.135" y="83.185" size="1.778" layer="91">Rele acelerador K2 NF</text>
<text x="-64.135" y="80.645" size="1.778" layer="91">Rele acelerador K2 C</text>
<text x="-71.755" y="78.105" size="1.778" layer="91">Central Botao Cambio K3 NF</text>
<text x="-64.135" y="75.565" size="1.778" layer="91">Comum Cambio K3 C</text>
<text x="-69.215" y="67.945" size="1.778" layer="91">Botao Cambio Frente K5 NF</text>
<text x="-69.215" y="65.405" size="1.778" layer="91">Botao Cambio Re K6 NF</text>
<text x="-53.34" y="0" size="1.778" layer="91">Brake Pedal </text>
<text x="-59.055" y="60.325" size="1.778" layer="91">Rele M/A K8 NF</text>
<text x="-59.055" y="57.785" size="1.778" layer="91">Rele M/A K8 C</text>
<text x="-43.815" y="55.245" size="1.778" layer="91">12 V</text>
<text x="-66.675" y="52.705" size="1.778" layer="91">Rele tecla de funcao K4 C</text>
<text x="-69.215" y="50.165" size="1.778" layer="91">Botao Cambio Frente K5 C</text>
<text x="-43.815" y="47.625" size="1.778" layer="91">5V</text>
<text x="-64.135" y="32.385" size="1.778" layer="91">Rele horn K3 C</text>
<text x="-64.135" y="29.845" size="1.778" layer="91">Rele horn K3 NA</text>
<text x="-64.135" y="27.305" size="1.778" layer="91">Rele low beam K4 C</text>
<text x="-64.135" y="24.765" size="1.778" layer="91">Rele low beam K4 NA</text>
<text x="-64.135" y="22.225" size="1.778" layer="91">Rele high beam K5 C</text>
<text x="-64.135" y="19.685" size="1.778" layer="91">Rele high beam K5 NA</text>
<text x="-64.135" y="17.145" size="1.778" layer="91">Rele right/left turn K6 C</text>
<text x="-64.135" y="14.605" size="1.778" layer="91">Rele right turn K6 NA</text>
<text x="-64.135" y="9.525" size="1.778" layer="91">Rele left turn K7 NA</text>
<text x="-23.495" y="60.325" size="1.778" layer="91">Botao Amarelo</text>
<text x="-22.86" y="0" size="1.778" layer="91">Freio signal/verde</text>
<text x="-23.495" y="85.725" size="1.778" layer="91">Sensor motor sinal (laranja)</text>
<text x="-23.495" y="88.265" size="1.778" layer="91">Sensor motor cosseno (amarelo)</text>
<text x="-23.495" y="90.805" size="1.778" layer="91">Sensor GND (marrom)</text>
<text x="-23.495" y="93.345" size="1.778" layer="91">Sensor do motor seno (vermelho)</text>
<text x="-23.495" y="108.585" size="1.778" layer="91">Rele Safe Stop</text>
<text x="-23.495" y="111.125" size="1.778" layer="91">12V/Botao_Amarelo</text>
<text x="-23.495" y="113.665" size="1.778" layer="91">GND (esp-socket)</text>
<text x="-23.495" y="116.205" size="1.778" layer="91">Azul direçao</text>
<text x="-23.495" y="118.745" size="1.778" layer="91">Verde direçao</text>
<text x="-23.495" y="121.285" size="1.778" layer="91">Vermelho direçao</text>
<text x="-68.58" y="118.745" size="1.778" layer="91">ODrive</text>
<text x="0" y="118.745" size="1.778" layer="91">Veiculo</text>
<text x="-31.75" y="0" size="1.778" layer="91">49</text>
<text x="-31.75" y="-2.54" size="1.778" layer="91">50</text>
<text x="-31.75" y="-5.08" size="1.778" layer="91">51</text>
<text x="-31.75" y="-7.62" size="1.778" layer="91">52</text>
<text x="-53.34" y="127.635" size="1.778" layer="91">Sindal inferior</text>
<text x="-22.86" y="127.635" size="1.778" layer="91">Sindal superior</text>
<text x="-23.495" y="47.625" size="1.778" layer="91">ESP Socket</text>
<text x="-55.88" y="-7.62" size="1.778" layer="91">Botoes safe stop</text>
<text x="-22.86" y="-7.62" size="1.778" layer="91">Lora</text>
<text x="-18.415" y="100.965" size="1.778" layer="91">--</text>
<text x="-18.415" y="98.425" size="1.778" layer="91">--</text>
<text x="-18.415" y="95.885" size="1.778" layer="91">--</text>
<text x="-22.86" y="-5.08" size="1.778" layer="91">GND freio laranja/ filtro do freio/Safe Stop/sensor vel./amarelo</text>
<text x="-48.26" y="-2.54" size="1.778" layer="91">K1E_C</text>
<text x="-22.86" y="-2.54" size="1.778" layer="91">Chave desativaçao freio</text>
<text x="-48.26" y="-5.08" size="1.778" layer="91">K1E_NA</text>
<text x="-43.815" y="103.505" size="1.778" layer="91">GND</text>
<text x="-58.42" y="132.715" size="6.4516" layer="91">TRAMONTINA</text>
<text x="-31.75" y="-10.16" size="1.778" layer="91">53</text>
<text x="-31.75" y="-12.7" size="1.778" layer="91">54</text>
<text x="-31.75" y="-15.24" size="1.778" layer="91">55</text>
<text x="-43.18" y="-15.24" size="1.778" layer="91">3.3V</text>
<text x="-31.75" y="-17.78" size="1.778" layer="91">56</text>
<text x="-31.75" y="-20.32" size="1.778" layer="91">57</text>
<text x="-31.75" y="-22.86" size="1.778" layer="91">58</text>
<text x="-31.75" y="-25.4" size="1.778" layer="91">59</text>
<text x="-31.75" y="-27.94" size="1.778" layer="91">60</text>
<text x="-31.75" y="-30.48" size="1.778" layer="91">61</text>
<text x="-31.75" y="-33.02" size="1.778" layer="91">62</text>
<text x="-31.75" y="-35.56" size="1.778" layer="91">63</text>
<text x="-31.75" y="-38.1" size="1.778" layer="91">64</text>
<text x="-22.86" y="-15.24" size="1.778" layer="91">VCC Freio (amarelo)/VCC Sensor Vel.(vermelho)</text>
<text x="-60.96" y="-35.56" size="1.778" layer="91">CB Atuador Linear +</text>
<text x="-60.96" y="-38.1" size="1.778" layer="91">CB Atuador Linear -</text>
<text x="-22.86" y="-35.56" size="1.778" layer="91">12V</text>
<text x="-22.86" y="-38.1" size="1.778" layer="91">GND</text>
<text x="-43.18" y="-12.7" size="1.778" layer="91">GND</text>
<text x="-68.58" y="-30.48" size="1.778" layer="91">Sensor Velocidade/amarelo</text>
<text x="-43.18" y="2.54" size="1.778" layer="91">5V </text>
<text x="-22.86" y="2.54" size="1.778" layer="91">5V Velocidade/marrom</text>
<text x="-68.58" y="-27.94" size="1.778" layer="91">Sensor Velocidade/branco</text>
<text x="-22.86" y="-27.94" size="1.778" layer="91">Sensor Velocidade/laranja</text>
<text x="-22.86" y="-30.48" size="1.778" layer="91">Sensor Velocidade/vermelho</text>
<text x="-73.66" y="43.18" size="1.778" layer="91">Rele leds verde/azul K12, K11 C</text>
<text x="-60.96" y="40.64" size="1.778" layer="91">Rele led azul K11 NA</text>
<text x="-63.5" y="38.1" size="1.778" layer="91">Rele led verde K12 NA</text>
<text x="-22.86" y="43.18" size="1.778" layer="91">24V</text>
<text x="-22.86" y="40.64" size="1.778" layer="91">Fio vermelho led azul</text>
<text x="-22.86" y="38.1" size="1.778" layer="91">Fio vermelho led verde</text>
<text x="-22.86" y="33.02" size="1.778" layer="91">Fio laranja - pos fusivel (buzina)</text>
<text x="-22.86" y="30.48" size="1.778" layer="91">Fio marrom - retorno (buzina)</text>
<text x="-22.86" y="17.78" size="1.778" layer="91">Vermelho - pos rele temporizador (77 diagrama veiculo)</text>
<text x="-22.86" y="15.24" size="1.778" layer="91">Verde - retorno seta direita (79 diagrama veiculo)</text>
<text x="-22.86" y="10.16" size="1.778" layer="91">Amarelo - retorno seta esquerda (78 diagrama veiculo)</text>
<text x="-23.495" y="52.705" size="1.778" layer="91">Cabo manga/cinza</text>
<text x="-23.495" y="50.165" size="1.778" layer="91">Cabo manga/azul</text>
<text x="-23.495" y="62.865" size="1.778" layer="91">Cinza controlbox</text>
<text x="-23.495" y="83.185" size="1.778" layer="91">Cabo manga/amarelo</text>
<text x="-23.495" y="80.645" size="1.778" layer="91">Cabo manga/vermelho</text>
<text x="-23.495" y="78.105" size="1.778" layer="91">Cabo manga/amarelo</text>
<text x="-23.495" y="75.565" size="1.778" layer="91">Cabo manga/verde</text>
<text x="-23.495" y="67.945" size="1.778" layer="91">Cabo manga/marrom</text>
<text x="-23.495" y="65.405" size="1.778" layer="91">Cabo manga/vermelho</text>
<text x="-23.495" y="55.88" size="1.778" layer="91">Alimentaçao placa TG</text>
<text x="-23.495" y="103.505" size="1.778" layer="91">GND (leds)</text>
</plain>
<instances>
<instance part="X1" gate="-1" x="-35.56" y="121.92" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-2" x="-25.4" y="121.92" smashed="yes"/>
<instance part="X1" gate="-3" x="-35.56" y="119.38" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-4" x="-25.4" y="119.38" smashed="yes"/>
<instance part="X1" gate="-5" x="-35.56" y="116.84" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-6" x="-25.4" y="116.84" smashed="yes"/>
<instance part="X1" gate="-7" x="-35.56" y="114.3" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-8" x="-25.4" y="114.3" smashed="yes"/>
<instance part="X1" gate="-9" x="-35.56" y="111.76" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-10" x="-25.4" y="111.76" smashed="yes"/>
<instance part="X1" gate="-11" x="-35.56" y="109.22" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-12" x="-25.4" y="109.22" smashed="yes"/>
<instance part="X1" gate="-13" x="-35.56" y="106.68" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-14" x="-25.4" y="106.68" smashed="yes"/>
<instance part="X1" gate="-15" x="-35.56" y="104.14" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-16" x="-25.4" y="104.14" smashed="yes"/>
<instance part="X1" gate="-17" x="-35.56" y="101.6" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-18" x="-25.4" y="101.6" smashed="yes"/>
<instance part="X1" gate="-19" x="-35.56" y="99.06" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-20" x="-25.4" y="99.06" smashed="yes"/>
<instance part="X1" gate="-21" x="-35.56" y="96.52" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-22" x="-25.4" y="96.52" smashed="yes"/>
<instance part="X1" gate="-23" x="-35.56" y="93.98" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-24" x="-25.4" y="93.98" smashed="yes"/>
<instance part="X1" gate="-25" x="-35.56" y="91.44" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-26" x="-25.4" y="91.44" smashed="yes"/>
<instance part="X1" gate="-27" x="-35.56" y="88.9" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-28" x="-25.4" y="88.9" smashed="yes"/>
<instance part="X1" gate="-29" x="-35.56" y="86.36" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-30" x="-25.4" y="86.36" smashed="yes"/>
<instance part="X1" gate="-31" x="-35.56" y="83.82" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-32" x="-25.4" y="83.82" smashed="yes"/>
<instance part="X1" gate="-33" x="-35.56" y="81.28" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-34" x="-25.4" y="81.28" smashed="yes"/>
<instance part="X1" gate="-35" x="-35.56" y="78.74" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-36" x="-25.4" y="78.74" smashed="yes"/>
<instance part="X1" gate="-37" x="-35.56" y="76.2" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-38" x="-25.4" y="76.2" smashed="yes"/>
<instance part="X1" gate="-39" x="-35.56" y="73.66" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-40" x="-25.4" y="73.66" smashed="yes"/>
<instance part="X1" gate="-41" x="-35.56" y="71.12" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-42" x="-25.4" y="71.12" smashed="yes"/>
<instance part="X1" gate="-43" x="-35.56" y="68.58" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-44" x="-25.4" y="68.58" smashed="yes"/>
<instance part="X1" gate="-45" x="-35.56" y="66.04" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-46" x="-25.4" y="66.04" smashed="yes"/>
<instance part="X1" gate="-47" x="-35.56" y="63.5" smashed="yes" rot="MR0"/>
<instance part="X1" gate="-48" x="-25.4" y="63.5" smashed="yes"/>
<instance part="X2" gate="-1" x="-35.56" y="60.96" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-2" x="-25.4" y="60.96" smashed="yes"/>
<instance part="X2" gate="-3" x="-35.56" y="58.42" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-4" x="-25.4" y="58.42" smashed="yes"/>
<instance part="X2" gate="-5" x="-35.56" y="55.88" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-6" x="-25.4" y="55.88" smashed="yes"/>
<instance part="X2" gate="-7" x="-35.56" y="53.34" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-8" x="-25.4" y="53.34" smashed="yes"/>
<instance part="X2" gate="-9" x="-35.56" y="50.8" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-10" x="-25.4" y="50.8" smashed="yes"/>
<instance part="X2" gate="-11" x="-35.56" y="48.26" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-12" x="-25.4" y="48.26" smashed="yes"/>
<instance part="X2" gate="-13" x="-35.56" y="45.72" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-14" x="-25.4" y="45.72" smashed="yes"/>
<instance part="X2" gate="-15" x="-35.56" y="43.18" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-16" x="-25.4" y="43.18" smashed="yes"/>
<instance part="X2" gate="-17" x="-35.56" y="40.64" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-18" x="-25.4" y="40.64" smashed="yes"/>
<instance part="X2" gate="-19" x="-35.56" y="38.1" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-20" x="-25.4" y="38.1" smashed="yes"/>
<instance part="X2" gate="-21" x="-35.56" y="35.56" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-22" x="-25.4" y="35.56" smashed="yes"/>
<instance part="X2" gate="-23" x="-35.56" y="33.02" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-24" x="-25.4" y="33.02" smashed="yes"/>
<instance part="X2" gate="-25" x="-35.56" y="30.48" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-26" x="-25.4" y="30.48" smashed="yes"/>
<instance part="X2" gate="-27" x="-35.56" y="27.94" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-28" x="-25.4" y="27.94" smashed="yes"/>
<instance part="X2" gate="-29" x="-35.56" y="25.4" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-30" x="-25.4" y="25.4" smashed="yes"/>
<instance part="X2" gate="-31" x="-35.56" y="22.86" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-32" x="-25.4" y="22.86" smashed="yes"/>
<instance part="X2" gate="-33" x="-35.56" y="20.32" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-34" x="-25.4" y="20.32" smashed="yes"/>
<instance part="X2" gate="-35" x="-35.56" y="17.78" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-36" x="-25.4" y="17.78" smashed="yes"/>
<instance part="X2" gate="-37" x="-35.56" y="15.24" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-38" x="-25.4" y="15.24" smashed="yes"/>
<instance part="X2" gate="-39" x="-35.56" y="12.7" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-40" x="-25.4" y="12.7" smashed="yes"/>
<instance part="X2" gate="-41" x="-35.56" y="10.16" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-42" x="-25.4" y="10.16" smashed="yes"/>
<instance part="X2" gate="-43" x="-35.56" y="7.62" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-44" x="-25.4" y="7.62" smashed="yes"/>
<instance part="X2" gate="-45" x="-35.56" y="5.08" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-46" x="-25.4" y="5.08" smashed="yes"/>
<instance part="X2" gate="-47" x="-35.56" y="2.54" smashed="yes" rot="MR0"/>
<instance part="X2" gate="-48" x="-25.4" y="2.54" smashed="yes"/>
<instance part="X3" gate="-41" x="-35.56" y="0" smashed="yes" rot="MR0"/>
<instance part="X3" gate="-42" x="-25.4" y="0" smashed="yes"/>
<instance part="X3" gate="-43" x="-35.56" y="-2.54" smashed="yes" rot="MR0"/>
<instance part="X3" gate="-44" x="-25.4" y="-2.54" smashed="yes"/>
<instance part="X3" gate="-45" x="-35.56" y="-5.08" smashed="yes" rot="MR0"/>
<instance part="X3" gate="-46" x="-25.4" y="-5.08" smashed="yes"/>
<instance part="X3" gate="-47" x="-35.56" y="-7.62" smashed="yes" rot="MR0"/>
<instance part="X3" gate="-48" x="-25.4" y="-7.62" smashed="yes"/>
<instance part="X4" gate="-47" x="-35.56" y="-10.16" smashed="yes" rot="MR0"/>
<instance part="X5" gate="-47" x="-35.56" y="-12.7" smashed="yes" rot="MR0"/>
<instance part="X6" gate="-47" x="-35.56" y="-15.24" smashed="yes" rot="MR0"/>
<instance part="X7" gate="-48" x="-25.4" y="-10.16" smashed="yes"/>
<instance part="X8" gate="-48" x="-25.4" y="-12.7" smashed="yes"/>
<instance part="X9" gate="-48" x="-25.4" y="-15.24" smashed="yes"/>
<instance part="X10" gate="-47" x="-35.56" y="-17.78" smashed="yes" rot="MR0"/>
<instance part="X11" gate="-48" x="-25.4" y="-17.78" smashed="yes"/>
<instance part="X12" gate="-47" x="-35.56" y="-20.32" smashed="yes" rot="MR0"/>
<instance part="X13" gate="-48" x="-25.4" y="-20.32" smashed="yes"/>
<instance part="X14" gate="-47" x="-35.56" y="-22.86" smashed="yes" rot="MR0"/>
<instance part="X15" gate="-48" x="-25.4" y="-22.86" smashed="yes"/>
<instance part="X16" gate="-47" x="-35.56" y="-25.4" smashed="yes" rot="MR0"/>
<instance part="X17" gate="-48" x="-25.4" y="-25.4" smashed="yes"/>
<instance part="X18" gate="-47" x="-35.56" y="-27.94" smashed="yes" rot="MR0"/>
<instance part="X19" gate="-48" x="-25.4" y="-27.94" smashed="yes"/>
<instance part="X20" gate="-47" x="-35.56" y="-30.48" smashed="yes" rot="MR0"/>
<instance part="X21" gate="-48" x="-25.4" y="-30.48" smashed="yes"/>
<instance part="X22" gate="-47" x="-35.56" y="-33.02" smashed="yes" rot="MR0"/>
<instance part="X23" gate="-48" x="-25.4" y="-33.02" smashed="yes"/>
<instance part="X24" gate="-47" x="-35.56" y="-35.56" smashed="yes" rot="MR0"/>
<instance part="X25" gate="-48" x="-25.4" y="-35.56" smashed="yes"/>
<instance part="X26" gate="-47" x="-35.56" y="-38.1" smashed="yes" rot="MR0"/>
<instance part="X27" gate="-48" x="-25.4" y="-38.1" smashed="yes"/>
</instances>
<busses>
</busses>
<nets>
<net name="N$1" class="0">
<segment>
<wire x1="-57.785" y1="116.205" x2="-57.15" y2="122.555" width="0.1524" layer="91" curve="-180"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<wire x1="-3.175" y1="122.555" x2="-3.175" y2="116.205" width="0.1524" layer="91" curve="-180"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
<compatibility>
<note version="8.2" severity="warning">
Since Version 8.2, EAGLE supports online libraries. The ids
of those online libraries will not be understood (or retained)
with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports URNs for individual library
assets (packages, symbols, and devices). The URNs of those assets
will not be understood (or retained) with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports the association of 3D packages
with devices in libraries, schematics, and board files. Those 3D
packages will not be understood (or retained) with this version.
</note>
</compatibility>
</eagle>
