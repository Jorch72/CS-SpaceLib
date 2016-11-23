// Wraps an IMyTextPanel instance to a class with additional functions. This class contains function to append text, print a line, print formatted text, centered text and more.
// Intended for use with the monospace font. Drop-in replacement for the VariableWidthTextPanel.
//
// Usage example:
//   var myTextPanel = new MonospaceTextPanel(textPanelBlock, 600); // use 600 for regular text panels and 1200 for wide text panels. Doesn't matter if the block is small or large.
//   myTextPanel.Reset(); // always call reset to clear the panel and prepare it
//   myTextPanel.PrintLn("Hello world!");
//   myTextPanel.PrintLn("Hi {0} {1:0.00}", "user", 5); // you can use formatting arguments, see https://msdn.microsoft.com/en-us/library/system.string.format(v=vs.110).aspx
public class MonospaceTextPanel
{
	private const double CHAR_WIDTH = 23;

	public IMyTextPanel Panel;
	public float FontSize;
	public float BaseWidth;
	public float ScaledWidth;

	// Width is 600 for regular text panels and 1200 for the wide text panel
	public MonospaceTextPanel(IMyTextPanel panel, float width)
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
		Panel.WritePublicText(new string(' ', (int)(pixels / CHAR_WIDTH / FontSize)), true);
	}

	public void PrintLnCentered(string text)
	{
		Indent(ScaledWidth - CHAR_WIDTH * text.Length * 0.5);
		PrintLn(text);
	}

	public void PrintLnCentered(string text, params object[] args)
	{
		PrintLnCentered(string.Format(text, args));
	}

	// Applies wrapping on long lines
	public void PrintWrapped(string text)
	{
		var charsPerLine = (int) (ScaledWidth / CHAR_WIDTH);
		var subStart = 0;
		while (subStart + charsPerLine < text.Length)
		{
			PrintLn(text.Substring(subStart, charsPerLine));
			subStart += charsPerLine;
		}
		Print(text.Substring(subStart));
	}

	public void PrintLnProgress(double ratio)
	{
		PrintLn(MakeProgressBar(ratio, ScaledWidth * FontSize));
	}

	public void PrintProgress(double ratio, double widthInPixels)
	{
		Print(MakeProgressBar(ratio, widthInPixels));
	}

	public string MakeProgressBar(double ratio, double width)
	{
		ratio = Math.Max(0, Math.Min(1, ratio));
		double barWidth = width / FontSize;
		int bars = Math.Max(0, (int)((barWidth * ratio * 2) / CHAR_WIDTH));
		int spaces = Math.Max(0, (int)((barWidth - (bars + 1) / 2 * CHAR_WIDTH) / CHAR_WIDTH));
		bool hasHalf = (bars % 2) == 1;
		return new string('¦', bars / 2) + (hasHalf ? "¦" : "") + new string(' ', spaces);
	}
}
