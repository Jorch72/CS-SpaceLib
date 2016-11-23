// Wraps an IMyTextPanel instance to a class with additional functions. This class contains function to append text, print a line, print formatted text, centered text and more.
//
// Usage example:
//   var myTextPanel = new VariableFontTextPanel(textPanelBlock, 600); // use 600 for regular text panels and 1200 for wide text panels. Doesn't matter if the block is small or large.
//   myTextPanel.Reset(); // always call reset to clear the panel and prepare it
//   myTextPanel.PrintLn("Hello world!");
//   myTextPanel.PrintLn("Hi {0} {1:0.00}", "user", 5); // you can use formatting arguments, see https://msdn.microsoft.com/en-us/library/system.string.format(v=vs.110).aspx
public class VariableFontTextPanel
{
	public IMyTextPanel Panel;
	public float FontSize;
	public float BaseWidth;
	public float ScaledWidth;

	// Width is 600 for regular text panels and 1200 for the wide text panel
	public VariableFontTextPanel(IMyTextPanel panel, float width)
	{
		Panel = panel;
		BaseWidth = width;
	}

	public void Reset()
	{
		Panel.WritePublicText("");
		Panel.ShowPublicTextOnScreen();
		FontSize = Panel.GetValue<float>("FontSize");
		ScaledWidth = BaseWidth / FontSize;
	}

	public void Print(string text)
	{
		Panel.WritePublicText(text, true);
	}

	public void Print(string text, params object[] args)
	{
		Panel.WritePublicText(args.Length == 0 ? text : string.Format(text, args), true);
	}

	public void PrintLn(string text)
	{
		Panel.WritePublicText(text + "\n", true);
	}

	public void PrintLn(string text, params object[] args)
	{
		Panel.WritePublicText((args.Length == 0 ? text : string.Format(text, args)) + '\n', true);
	}

	public void Indent(double pixels)
	{
		Panel.WritePublicText(new string(' ', (int)(pixels / StringFunctions.WHITESPACE_WIDTH / FontSize)), true);
	}

	public void PrintLnCentered(string text)
	{
		Indent((ScaledWidth - StringFunctions.MeasureStringLength(text)) * 0.5f);
		PrintLn(text);
	}

	public void PrintLnCentered(string text, params object[] args)
	{
		PrintLnCentered(string.Format(text, args));
	}

	// Performs multi-line wrapping if the string is long.
	public void PrintWrapped(string text)
	{
		var lengthSoFar = 0f;
		var subStart = 0;
		for (int subEnd = 0; subEnd < text.Length; subEnd++)
		{
			lengthSoFar += StringFunctions.GetCharSize(text[subEnd]);
			if (lengthSoFar > ScaledWidth)
			{
				PrintLn(text.Substring(subStart, subEnd - subStart));
				subStart = subEnd;
				lengthSoFar = StringFunctions.GetCharSize(text[subEnd]);
			}
		}

		Print(text.Substring(subStart));
	}

	// Prints a progress bar. Ratio is a number in the range [0, 1]. The progress bar fill the entire width of the panel.
	public void PrintLnProgress(double ratio)
	{
		PrintLn(MakeProgressBar(ratio, ScaledWidth * FontSize));
	}
	
	// Prints a progress bar. Ratio is a number in the range [0, 1].
	public void PrintProgress(double ratio, double widthInPixels)
	{
		Print(MakeProgressBar(ratio, widthInPixels));
	}
	
	// Creates a progress bar string using spaces and pipe (|) characters.
	public string MakeProgressBar(double ratio, double width)
	{
		ratio = Math.Max(0f, Math.Min(1f, ratio));
		double barWidth = width / FontSize - 18f;
		int bars = Math.Max(0, (int)((barWidth * ratio) / StringFunctions.PROGRESSCHAR_WIDTH));
		int spaces = Math.Max(0, (int)((barWidth - bars * StringFunctions.PROGRESSCHAR_WIDTH) / StringFunctions.WHITESPACE_WIDTH));
		return "[" + new string('|', bars) + new string(' ', spaces) + ']';
	}
}


// Credits to MMaster for the char sizes
// Credits to DarkInferno for his version
// Credits to myself (Stan) for further optimizations and minor additions
public static class StringFunctions
{
	private static Dictionary<char, float> charSize;

	public const float WHITESPACE_WIDTH = 8f;
	public const float PROGRESSCHAR_WIDTH = 6.4f;

	private static float ELLIPSIS_WIDTH = 31f;

	static StringFunctions()
	{
		charSize = new Dictionary<char, float>();

		AddCharsSize("3FKTabdeghknopqsuy£µÝàáâãäåèéêëðñòóôõöøùúûüýþÿaaaddeeeeegggghhKknnn?ooosssšTTTuuuuuuYyŸ???????????????????", 17f);
		AddCharsSize("#0245689CXZ¤¥ÇßCCCCZZŽƒ???????????????€", 19f);
		AddCharsSize("$&GHPUVY§ÙÚÛÜÞAGGGGHHUUUUUU???????†‡", 20f);
		AddCharsSize("ABDNOQRSÀÁÂÃÄÅÐÑÒÓÔÕÖØAADÐNNNOOORRRSSSŠ??????", 21f);
		AddCharsSize("(),.1:;[]ft{}·ttt?", 9f);
		AddCharsSize("+<=>E^~¬±¶ÈÉÊË×÷EEEEE?????-", 18f);
		AddCharsSize(" !I`ijl¡¨¯´¸ÌÍÎÏìíîïIiIiIiIijllllˆ???°?˜?????‹›·", 8f);
		AddCharsSize("7?Jcz¢¿çccccJzzž????????????????", 16f);
		AddCharsSize("L_vx«»LLL?L???????–•", 15f);
		AddCharsSize("\"-rª­º?rrr", 10f);
		AddCharsSize("mw¼w??", 27f);
		AddCharsSize("M??", 26f);
		AddCharsSize("WÆŒW—…‰", 31f);
		AddCharsSize("'|¦¯‘’‚", 6f);
		AddCharsSize("*²³¹", 11f);
		AddCharsSize("\\°“”„", 12f);
		AddCharsSize("/????", 14f);
		AddCharsSize("%??", 24f);
		AddCharsSize("@©®???", 25f);
		AddCharsSize("\n", 0f);
		AddCharsSize("¾æœ?", 28f);
		AddCharsSize("½?", 29f);
		AddCharsSize("?", 7f);
		AddCharsSize("?", 22f);
		AddCharsSize("?", 13f);
		AddCharsSize("?", 23f);
		AddCharsSize("™", 30f);
	}

	private static void AddCharsSize(string chars, float size)
	{
		for (int i = 0; i < chars.Length; i++)
			charSize.Add(chars[i], size / 0.8f); // measurements were made by MMaster for font size = 0.8, we want it normalize for font size = 1.0
	}

	public static float GetCharSize(char c)
	{
		float width = 40;
		charSize.TryGetValue(c, out width);
		return width;
	}

	// Returns the size of a given string
	public static float MeasureStringLength(string str)
	{
		float sum = 0;
		for (int i = 0; i < str.Length; i++)
			sum += GetCharSize(str[i]);

		return sum;
	}

	// If the string too long, chop it off and append ...
	public static string Ellipsize(string text, float pixelWidth)
	{
		float stringSize = MeasureStringLength(text);
		if (stringSize <= pixelWidth)
			return text;

		stringSize += ELLIPSIS_WIDTH;
		int trimTo = text.Length;

		while (stringSize > pixelWidth)
		{
			trimTo--;
			stringSize -= GetCharSize(text[trimTo]);
		}

		return text.Substring(0, trimTo) + "…";
	}
}
