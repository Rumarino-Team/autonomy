Result<Option<AggregateType::Field>, ReflectionError> ProcessField(
    usize field_offset,
    std::string_view field_name,
    usize pointer_size,
    const Type* const* field_type,
    TypeDatabase::TypeList& types
)
{
    const usize pointer_count = CountPointers(field_name);
    const usize name_length = CalculateNameLength(field_name, pointer_count);
    const std::string_view name = std::string_view(&field_name[pointer_count], name_length);
    const Type* const* type = field_type;

    if (!IsValidName(name))
    {
        return MakeError(ReflectionError::InvalidSdnaFieldName);
    }

    usize name_end = pointer_count + name_length;
    std::vector<usize> dimensions;

    // Parse every bracket. Upstream consumes a whole multidimensional suffix and
    // keeps only the first count, so later DNA fields land at the wrong offset.
    while (name_end < field_name.size())
    {
        if (field_name[name_end] != '[')
        {
            return MakeError(ReflectionError::InvalidSdnaFieldName);
        }

        const auto array_begin = name_end + 1;
        const auto array_end = field_name.find(']', array_begin);
        if (array_end == std::string_view::npos || array_end == array_begin)
        {
            return MakeError(ReflectionError::InvalidSdnaFieldName);
        }

        usize count = 0;
        const auto result = std::from_chars(field_name.data() + array_begin, field_name.data() + array_end, count);
        if (result.ec != std::errc() || result.ptr != field_name.data() + array_end)
        {
            return MakeError(ReflectionError::InvalidSdnaFieldName);
        }

        dimensions.push_back(count);
        name_end = array_end + 1;
    }

    // C declaration order: name[A][B] is an array of A arrays of B elements.
    for (auto dimension = dimensions.rbegin(); dimension != dimensions.rend(); ++dimension)
    {
        types.push_back(MakeContainer<ArrayType>(*dimension, type));
        type = &types.back();
    }

    return AggregateType::Field{ .offset = field_offset, .name = name, .type = AddPointers(pointer_count, type, pointer_size, types) };
}
