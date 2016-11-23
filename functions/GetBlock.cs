// Retrieves a block with the given name and casts it. Warns if the block is missing.
public T GetBlock<T>(string name) where T : class, IMyTerminalBlock
{
	var result = GridTerminalSystem.GetBlockWithName(name) as T;
	if (result == null)
		Echo("Could not find " + name);
	return result;
}
