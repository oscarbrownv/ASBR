function res = m_isequal(a, b)
    tol = 1e-6;
    res = norm(a-b, "fro") < tol;
end