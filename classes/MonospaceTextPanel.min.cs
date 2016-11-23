// See MonospaceTextPanel.cs
public class MonospaceTextPanel {
private const double C=23;public IMyTextPanel Panel;public float FontSize,BaseWidth,ScaledWidth;
public MonospaceTextPanel(IMyTextPanel p,float w){Panel=p;BaseWidth=w;}
public void Reset(){Panel.WritePublicText("");Panel.ShowPublicTextOnScreen();FontSize=Panel.GetValue<float>("FontSize");ScaledWidth=BaseWidth/FontSize;}
public void Print(string t){Panel.WritePublicText(t, true);}
public void Print(string t,params object[] a){Panel.WritePublicText(a.Length==0?t:string.Format(t,a),true);}
public void PrintLn(string t){Panel.WritePublicText(t+"\n",true);}
public void PrintLn(string t, params object[]a){Panel.WritePublicText((a.Length==0?t:string.Format(t,a))+'\n',true);}
public void Indent(double p){Panel.WritePublicText(new string(' ',(int)(p/C/FontSize)),true);}
public void PrintLnCentered(string t){Indent(ScaledWidth-C*t.Length*0.5);PrintLn(t);}
public void PrintLnCentered(string t, params object[]a){PrintLnCentered(string.Format(t, a));}
public void PrintWrapped(string t){var a=(int)(ScaledWidth/C);var b=0;while(b+a<t.Length){PrintLn(t.Substring(b,a));b+=a;}Print(t.Substring(b));}
public void PrintLnProgress(double r){PrintLn(MakeProgressBar(r,ScaledWidth*FontSize));}
public void PrintProgress(double r, double w){Print(MakeProgressBar(r,w));}
public string MakeProgressBar(double r, double w){r=Math.Max(0,Math.Min(1,r));var a=w/FontSize;int b=Math.Max(0,(int)((a*r*2)/C));int c=Math.Max(0,(int)((a-(b+1)/2*C)/C));bool d=(b%2)==1;return new string('¦',b/2)+(d?"¦":"")+new string(' ',c);}
}
