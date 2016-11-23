// See GetBlock.cs
public T GetBlock<T>(string n) where T:class,IMyTerminalBlock{var r=GridTerminalSystem.GetBlockWithName(n) as T;if(r==null)Echo("Could not find "+n);return r;}
