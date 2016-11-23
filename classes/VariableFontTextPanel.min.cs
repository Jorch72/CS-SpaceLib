// See VariableFontTextPanel.cs
public class VariableFontTextPanel {
public IMyTextPanel Panel;public float FontSize,BaseWidth,ScaledWidth;
public VariableFontTextPanel(IMyTextPanel p,float w){Panel=p;BaseWidth=w;}
public void Reset(){Panel.WritePublicText("");Panel.ShowPublicTextOnScreen();FontSize=Panel.GetValue<float>("FontSize");ScaledWidth=BaseWidth/FontSize;}
public void Print(string t){Panel.WritePublicText(t,true);}
public void Print(string t,params object[]a){Panel.WritePublicText(a.Length==0?t:string.Format(t,a),true);}
public void PrintLn(string t){Panel.WritePublicText(t+"\n",true);}
public void PrintLn(string t, params object[]a){Panel.WritePublicText((a.Length==0?t:string.Format(t,a))+'\n',true);}
public void Indent(double p){Panel.WritePublicText(new string(' ',(int)(p/StringFunctions.WHITESPACE_WIDTH/FontSize)),true);}
public void PrintLnCentered(string t){Indent((ScaledWidth-StringFunctions.MeasureStringLength(t))*0.5f);PrintLn(t);}
public void PrintLnCentered(string t,params object[]a){PrintLnCentered(string.Format(t, a));}
public void PrintWrapped(string t){var a=0f;var b=0;for(int c=0;c<t.Length;c++){a+=StringFunctions.GetCharSize(t[c]);if(a>ScaledWidth){PrintLn(t.Substring(b,c-b));b=c;a=StringFunctions.GetCharSize(t[c]);}}Print(t.Substring(b));}
public void PrintLnProgress(double r){PrintLn(MakeProgressBar(r,ScaledWidth*FontSize));}
public void PrintProgress(double r, double w){Print(MakeProgressBar(r, w));}
public string MakeProgressBar(double r, double w){r=Math.Max(0f,Math.Min(1f,r));var a=w/FontSize-18f;int bars=Math.Max(0,(int)((a*r)/StringFunctions.PROGRESSCHAR_WIDTH));int b=Math.Max(0,(int)((a-bars*StringFunctions.PROGRESSCHAR_WIDTH)/StringFunctions.WHITESPACE_WIDTH));return"["+new string('|',bars)+new string(' ',b)+']';}
}

public static class StringFunctions {
private static Dictionary<char, float> cs;
public const float WHITESPACE_WIDTH=8f,PROGRESSCHAR_WIDTH=6.4f,ELLIPSIS_WIDTH = 31f;
static StringFunctions(){cs=new Dictionary<char,float>();A("3FKTabdeghknopqsuy£µÝàáâãäåèéêëðñòóôõöøùúûüýþÿaaaddeeeeegggghhKknnn?ooosssšTTTuuuuuuYyŸ???????????????????",17);A("#0245689CXZ¤¥ÇßCCCCZZŽƒ???????????????€",19);A("$&GHPUVY§ÙÚÛÜÞAGGGGHHUUUUUU???????†‡",20);A("ABDNOQRSÀÁÂÃÄÅÐÑÒÓÔÕÖØAADÐNNNOOORRRSSSŠ??????",21);A("(),.1:;[]ft{}·ttt?",9);A("+<=>E^~¬±¶ÈÉÊË×÷EEEEE?????-",18);A(" !I`ijl¡¨¯´¸ÌÍÎÏìíîïIiIiIiIijllllˆ???°?˜?????‹›·",8);A("7?Jcz¢¿çccccJzzž????????????????",16);A("L_vx«»LLL?L???????–•",15);A("\"-rª­º?rrr",10);A("mw¼w??",27);A("M??",26);A("WÆŒW—…‰",31);A("'|¦¯‘’‚",6);A("*²³¹", 11);A("\\°“”„",12);A("/????",14);A("%??",24);A("@©®???", 25);A("\n",0);A("¾æœ?",28);A("½?",29);A("?",7);A("?", 22);A("?",13);A("?",23);A("™",30);}
private static void A(string c,float s){for(int i=0;i<c.Length;i++)cs.Add(c[i],s/0.8f);}
public static float GetCharSize(char c){float w=40;cs.TryGetValue(c,out w);return w;}
public static float MeasureStringLength(string s){float a=0;for(int i=0;i<s.Length;i++)a+=GetCharSize(s[i]);return a;}
public static string Ellipsize(string t,float p){var a=MeasureStringLength(t);if(a<=p)return t;a+=ELLIPSIS_WIDTH;int b=t.Length;while(a>p){b--;a-=GetCharSize(t[b]);}return t.Substring(0,b)+"…";}
}